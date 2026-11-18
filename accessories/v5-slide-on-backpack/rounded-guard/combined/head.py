"""Registered head delta for the concealed FACE and co-printed toothed jaws (beard and mask off)."""
import hashlib
import json
from pathlib import Path
import runpy
import sys

import numpy as np


HERE=Path(__file__).resolve().parent
ACC=HERE.parents[2]
HARDWARE=ACC.parent
RUNTIME=HARDWARE.parent/'xgoduck_runtime_arduino'
PROFILE=HERE/'head-profile.json'
PREFIX='combined_head_'


def generate():
    from scipy.spatial import ConvexHull
    beak=ACC/'v5-dino-beak'
    magnetic=ACC/'v5-face-plates'/'magnetic'
    co=runpy.run_path(str(beak/'coprint'/'coprint.py'))
    brow=runpy.run_path(str(magnetic/'brow.py'))
    preview=runpy.run_path(str(beak/'export-preview.py'))
    props=runpy.run_path(str(ACC/'v5-raptor-tail'/'tail_physics.py'))
    list(preview['stock_head'](RUNTIME))
    jaws={s:co['build'](s) for s in ('upper','lower')}
    chin=co['CHIN']['build'](jaws['lower']['rigid'])
    face_to_upper=((0,-1,0,0),(0,0,1,brow['BASE']['FACE_Y']),(-1,0,0,33.258))
    stock_face=brow['CAD']['read_stock']('FACE')
    face_hardware=[('face_target_keepers',brow['rear_keepers'](),1200),
                   ('face_steel_targets',brow['target_discs'](),7850)]
    solids=[('face_pockets','upper',(stock_face-brow['magnetic_face']()).transform(face_to_upper),-1200),
            ('upper_key_recesses','upper',jaws['upper']['stock']-jaws['upper']['rigid'],-1200),
            ('lower_keys_and_hatch','lower',jaws['lower']['stock']-chin['host'],-1200),
            ('chin_keeper','lower',chin['lid'],1200),('chin_steel_targets','lower',chin['steel'],7850)]
    for name,part,density in face_hardware:
        piece=brow['to_print'](part)
        assert (piece-stock_face).volume()<.0001
        solids.append((name,'upper',piece.transform(face_to_upper),density))
    hulls={}
    def register(pts,side):
        return preview['registered'](pts,side)/1000
    for side,r in jaws.items():
        solids.append((side+'_tpu',side,r['soft'],1200))
        for i,tooth in enumerate(r['teeth']):
            key=f"tooth_{side}_{'plus' if i%2==0 else 'minus'}_{i//2+1}"
            # Ignore the buried T-root. The stock head already supplies the
            # rigid shell's contact; one convex hull per exposed soft tooth
            # preserves all gaps between adjacent teeth.
            exposed=tooth-r['stock']
            xyz=register(exposed.to_mesh().vert_properties[:,:3],side)
            verts=xyz[ConvexHull(xyz).vertices]
            hulls[key]=verts.round(9).tolist()
    assert len(hulls)==16
    parts={}
    masses={}
    for name,side,solid,signed_density in solids:
        tri=register(props['triangles'](solid),side)
        density=abs(signed_density)
        mass,com,I=props['mass_properties'](tri,np.full(len(tri),density))
        assert abs(1e9*mass/density-solid.volume())<.001
        sign=1 if signed_density>0 else -1
        parts[name]=dict(sign=sign,mass_kg=mass,com_m=com.tolist(),inertia_kg_m2=I.tolist())
        masses[name]=sign*mass*1000
    mass=sum(masses.values())/1000
    com=sum(p['mass_kg']*p['sign']*np.asarray(p['com_m']) for p in parts.values())/mass
    def shift(v):return np.eye(3)*(v@v)-np.outer(v,v)
    I=sum(p['sign']*(np.asarray(p['inertia_kg_m2'])+
                     p['mass_kg']*shift(np.asarray(p['com_m'])-com)) for p in parts.values())
    files=[beak/'coprint'/'coprint.py',beak/'coprint'/'print'/'head_bottom-keyed-teeth.3mf',
           beak/'coprint'/'print'/'jaw-keyed-teeth-magnetic-hatch.3mf',magnetic/'chin.py',magnetic/'brow.py',
           RUNTIME/'assets/sim/model/assets/head_roll.stl']
    return dict(parent='jaw_soft',mode='coprinted_teeth_magnetic_receivers',neutral_jaw=True,
                mask_and_beard='removed',cheek_screws_and_nuts=0,
                contact_assumption='One rigid convex proxy per exposed TPU tooth, stock head-to-floor and head-to-trunk contact filters.',
                mass_kg=mass,delta_mass_g=mass*1000,com_m=com.tolist(),inertia_kg_m2=I.tolist(),
                densities_kg_m3={'rigid':1200,'tpu':1200,'steel':7850},masses_g=masses,
                parts=parts,hulls_m=hulls,
                sha256={p.name:hashlib.sha256(p.read_bytes().replace(b'\r\n',b'\n') if p.suffix=='.py'
                                             else p.read_bytes()).hexdigest() for p in files})


def configure(spec,profile=None):
    """Apply this whole head delta once to the stock jaw_soft body."""
    import mujoco as mj
    if profile is None:
        profile=json.loads(PROFILE.read_text(encoding='utf8'))
    body=spec.body(profile['parent'])
    receiver=runpy.run_path(str(HERE.parents[1]/'removable-rails'/'receiver_physics.py'))
    # Use the signed aggregate only as a correction to the physical stock
    # inertia. A delta tensor alone need not represent a physical free body.
    receiver['subtract'](body,-profile['mass_kg'],profile['com_m'],-np.array(profile['inertia_kg_m2']))
    for name,vertices in profile['hulls_m'].items():
        mesh=PREFIX+name
        spec.add_mesh(name=mesh,uservert=np.ravel(vertices).tolist())
        body.add_geom(name=mesh,type=mj.mjtGeom.mjGEOM_MESH,meshname=mesh,
                      mass=0,contype=3,conaffinity=0,group=3)


def verify_compiled():
    import mujoco as mj
    sys.path.insert(0,str(RUNTIME/'python'))
    from walk_sim import load_recovery_scene
    profile=json.loads(PROFILE.read_text(encoding='utf8'))
    baseline=load_recovery_scene(mj)
    model=load_recovery_scene(mj,configure)
    data=mj.MjData(model)
    mj.mj_forward(model,data)
    name=profile['parent'];bid=model.body(name).id
    delta=float(sum(model.body_mass)-sum(baseline.body_mass))
    assert abs(delta-profile['mass_kg'])<1e-8
    assert model.ngeom-baseline.ngeom==16
    assert all(n not in profile['parts'] for n in ('upper_four_square_nuts','lower_four_square_nuts'))
    errors=[]
    for label,points in profile['hulls_m'].items():
        g=model.geom(PREFIX+label).id
        assert model.geom_bodyid[g]==bid
        assert (model.geom_contype[g],model.geom_conaffinity[g])==(3,0)
        mesh=model.geom_dataid[g]
        start=model.mesh_vertadr[mesh]
        local=model.mesh_vert[start:start+model.mesh_vertnum[mesh]]
        world=local@data.geom_xmat[g].reshape(3,3).T+data.geom_xpos[g]
        compiled=(world-data.xpos[bid])@data.xmat[bid].reshape(3,3)
        # Principal-axis reorientation performed by the MuJoCo mesh compiler
        # must preserve the CAD vertex cloud in the parent body's frame.
        distances=np.linalg.norm(compiled[:,None,:]-np.asarray(points)[None,:,:],axis=-1)
        errors.append(max(distances.min(0).max(),distances.min(1).max()))
    assert max(errors)<1e-6
    for _ in range(20):mj.mj_step(model,data)
    assert np.isfinite(data.qpos).all()
    print(f'Head delta {delta*1000:.6f} g, 16 compiled hulls; largest vertex registration error {max(errors)*1000:.6f} mm')


if __name__=='__main__':
    if sys.argv[1:]==['--check']:
        verify_compiled()
    elif not sys.argv[1:]:
        p=generate()
        PROFILE.write_text(json.dumps(p,separators=(',',':'),allow_nan=False)+'\n',encoding='utf8')
        print(f"{p['delta_mass_g']:.6f} g, {len(p['hulls_m'])} tooth hulls")
    else:
        raise SystemExit('Usage: head.py [--check]')
