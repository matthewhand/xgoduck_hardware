"""Build the locked/passive MuJoCo variants from the seated V5 tail CAD.

The profile generated once from CAD can be loaded without CAD dependencies.
All body origins, inertia tensors and collider vertices are expressed in SI.
"""
import argparse
import json
from pathlib import Path
import runpy

import numpy as np

HERE = Path(__file__).resolve().parent
PITCH = 'passive_tail_pitch'
YAW = 'passive_tail_yaw'
PARTS = ('mount','proximal','distal')
VARIANTS = ('bare','locked','passive')


def trunk(points):
    p = np.asarray(points)
    return (p[..., [2,0,1]] - [57.33226,.00628,39.25512]) / 1000


def triangles(solid):
    mesh = solid.to_mesh()
    return mesh.vert_properties[mesh.tri_verts,:3].astype(float)


def mass_properties(tris, density):
    # Integrate signed tetrahedra about the local body origin, then transfer
    # the second moment to the combined printed-and-hardware centre of mass.
    a,b,c = tris.transpose(1,0,2)
    dm = density * np.einsum('ij,ij->i', a, np.cross(b,c)) / 6
    sums = a+b+c
    mass = dm.sum()
    first = dm @ sums / 4
    second = (np.einsum('i,ij,ik->jk',dm,sums,sums)
              + np.einsum('i,ikj,ikl->jl',dm,tris,tris)) / 20
    com = first/mass
    centered = second - mass*np.outer(com,com)
    inertia = np.eye(3)*np.trace(centered) - centered
    if mass<=0 or min(np.linalg.eigvalsh(inertia)) <= 0:
        raise ValueError('Non-positive CAD mass or principal inertia')
    return float(mass),com,inertia


def collider_vertices(solid, origin, group):
    # Section each component first so a single convex hull cannot fill the
    # complete pack opening or join the tail root directly to the tail tip.
    from scipy.spatial import ConvexHull
    directions = []
    for i in range(64):
        v = 1-2*(i+.5)/64
        r = np.sqrt(1-v*v)
        a = i*np.pi*(3-np.sqrt(5))
        directions.append((r*np.cos(a),r*np.sin(a),v))
    axes = np.vstack((directions,np.eye(3),-np.eye(3)))
    direction = np.array([1,0,0]) if group=='mount' else np.array([0,0,1])
    hulls = []
    for part in solid.decompose():
        if part.volume()<.01:
            continue
        lo,hi = np.array(part.bounding_box()).reshape(2,3) @ direction
        cuts = np.linspace(lo-.01,hi+.01,max(2,int(np.ceil((hi-lo)/14))+1))
        for lower,upper in zip(cuts,cuts[1:]):
            piece = part.trim_by_plane(direction,lower).trim_by_plane(-direction,-upper)
            if piece.volume()<.1:
                continue
            vertices = trunk(piece.to_mesh().vert_properties[:,:3]) - origin
            support = np.unique(vertices[(vertices@axes.T).argmax(0)],axis=0)
            hulls.append(np.round(support[ConvexHull(support).vertices],7).tolist())
    return hulls


def generate_profile(length='standard'):
    generator = runpy.run_path(str(HERE/'raptor-tail.py'))
    parts = generator['components'](length)
    layout = json.loads((HERE/'tail-layout.json').read_text(encoding='utf-8'))
    joints = layout['joints']
    estimate = layout['estimate' if length=='standard' else 'short_estimate']
    pivots = [np.zeros(3),*(trunk(j['origin_mm']) for j in joints)]
    bodies = {}
    for k, name in enumerate(PARTS):
        tris,densities,hulls = [],[],[]
        for label,(group,part) in parts.items():
            if group!=name:
                continue
            vertices = trunk(triangles(part))-pivots[k]
            tris.append(vertices)
            kg = estimate['parts'][label]['mass_g']['nominal']/1000
            densities.append(np.full(len(vertices),kg/(part.volume()*1e-9)))
            hulls.extend(collider_vertices(part,pivots[k],name))
        mass,com,inertia = mass_properties(np.vstack(tris),np.concatenate(densities))
        assert abs(1000*mass-estimate['groups'][name]['mass_g']['nominal']) < .001
        bodies[name] = dict(origin_m=pivots[k].tolist(),mass_kg=mass,com_m=com.tolist(),
                            inertia_kg_m2=inertia.tolist(),
                            collider_hulls_m=hulls)
    return dict(units='SI',parent='trunk_base',bodies=bodies,
                hinge_limit_deg=joints[0]['range_deg'][1],hinge_axes=[np.array(j['axis'])[[2,0,1]].tolist() for j in joints],
                damping_nm_s_rad=[.00032,.00018],friction_nm=[.00008,.00008],
                mass_source='tail-layout.json, per-part nominal print/fastener estimates')


def read_profile():
    return json.loads((HERE/'tail-physics.json').read_text(encoding='utf-8'))


def configure(spec, profile, variant, drag=1.0):
    import mujoco as mj
    if variant not in VARIANTS or not drag>0:
        raise ValueError('Unexpected tail variant or drag multiplier')
    if variant=='bare':
        return
    parent = spec.body('trunk_base')
    origin = np.zeros(3)
    for index,name in enumerate(PARTS):
        p = profile['bodies'][name]
        current = np.array(p['origin_m'])
        b = parent.add_body(name='tail_'+name,pos=current-origin)
        b.explicitinertial = True
        b.mass = p['mass_kg']
        b.ipos = p['com_m']
        inertia = np.array(p['inertia_kg_m2'])
        b.fullinertia = inertia[[0,1,2,0,0,1],[0,1,2,1,2,2]]
        if variant=='passive' and index:
            j=index-1
            # All source runtime/training scenes use radians.
            limit = np.deg2rad(profile['hinge_limit_deg'])
            b.add_joint(name=(PITCH,YAW)[j],type=mj.mjtJoint.mjJNT_HINGE,
                        axis=profile['hinge_axes'][j],limited=True,range=[-limit,limit],
                        stiffness=0,armature=0,damping=profile['damping_nm_s_rad'][j]*drag,
                        frictionloss=profile['friction_nm'][j]*drag)
        for i,vertices in enumerate(p['collider_hulls_m']):
            mesh = f'tail_{name}_{i}'
            spec.add_mesh(name=mesh,uservert=np.asarray(vertices).ravel().tolist())
            b.add_geom(name=mesh+'_collision',type=mj.mjtGeom.mjGEOM_MESH,meshname=mesh,
                       mass=0,contype=1,conaffinity=0,group=3)
        parent,origin=b,current


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('output',type=Path)
    p.add_argument('--length',choices=('standard','short'),default='standard')
    args=p.parse_args()
    profile=generate_profile(args.length)
    args.output.write_text(json.dumps(profile,separators=(',',':'))+'\n',encoding='utf-8')
    print('CAD added mass (g):',sum(p['mass_kg'] for p in profile['bodies'].values())*1000)
