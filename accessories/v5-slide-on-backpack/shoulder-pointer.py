"""Right-shoulder three-lobe feeler, in seated V5 enclosure STL millimetres."""
import argparse
import json
from pathlib import Path
import runpy
import numpy as np
from manifold3d import Manifold as M, OpType


# Internal drawing axes are trunk forward, left and up; distances are mm.
OFFSET = np.array([57.33226, .00628, 39.25512])
ROOT = np.array([-15., -26., 50.20]) - OFFSET
SPINE = np.array([ROOT+[0,0,4], [-65,-40,23], [-38,-51,34],
                  [18,-55,40], [63,-56,53], [96,-56,69], [119,-55,77.]])
PAD = np.array([132.,-55.,82.])
TOUCH = np.array([144.,-55.,82.])
HUB = np.array([117.,-55.,77.0])
OPEN_DEG = 38
POSES = {"closed":0, "open":OPEN_DEG}


def ball(p, r):
    return M.sphere(r,20).translate(p)


def bridge(a,b,r1,r2=None):
    return M.batch_hull([ball(a,r1),ball(b,r1 if r2 is None else r2)])


def union(solids):
    return M.batch_boolean(solids,OpType.Add)


def cyl(p,r,h):
    return M.cylinder(h,r,r,24).translate(p)


def segments(pose):
    """Conservative convex ground/body contact segments in trunk mm."""
    angle=np.deg2rad(POSES[pose])
    values=[("stem",p,q,r) for p,q,r in zip(SPINE,SPINE[1:],[3.6,3.5,3.5,3.3,3.1,3.0])]
    values.extend([("central",SPINE[-1],PAD,2.7),("central",PAD,TOUCH,2.7)])
    for sign in [-1,1]:
        pivot=HUB+np.array([0,sign*5.5,0])
        end=pivot+np.array([18*np.cos(angle),sign*18*np.sin(angle),4.4])
        values.extend([("hub",HUB,pivot,2.6),("wing",pivot+np.array([0,0,2.9]),end,2.6)])
    return values


def trunk_parts(pose="closed"):
    """Parts ready to assemble. The outer shoes rotate manually about M2 screws."""
    segments(pose)  # validate pose
    base=M.cube([11,6.4,3.3]).translate(ROOT+[-5.5,-3.2,-.65])
    rigid=[base,*[bridge(p,q,r) for kind,p,q,r in segments(pose) if kind=="stem"],
           bridge(SPINE[-1],PAD,2.9,2.65),bridge(PAD,TOUCH-[4.7,0,0],2.65,1.8)]
    # A reduced rigid core and shoulder keep the soft centre pad registered.
    rigid.append(bridge(TOUCH-[5.,0,0],TOUCH-[1.35,0,0],1.45,1.15))
    hub_notes=[]
    for sign in [-1,1]:
        pivot=HUB+[0,sign*5.5,0]
        rigid.append(bridge(HUB-[1.3,0,0],pivot,2.25))
        rigid.append(cyl(pivot+[0,0,-1.9],3.4,3.8))
        hub_notes.append(pivot)
    stem=union(rigid)
    for pivot in hub_notes:
        stem-=cyl(pivot-[0,0,5],1.15,11)
    parts={"stem":stem,"soft_tip":bridge(TOUCH-[3.,0,0],TOUCH,2.75,3.6)-stem}
    for sign,pivot in zip([-1,1],hub_notes):
        side="outboard" if sign<0 else "inboard"
        direction=np.array([1,0,0])
        end=pivot+18*direction+[0,0,4.4]
        # Flat hinge under the rounded lobe, 0.30 mm axial gap to the rigid base.
        wing=cyl(pivot+[0,0,2.55],3.65,2.5)
        wing+=bridge(pivot+3.3*direction+[0,0,3.8],end,1.4,2.65)
        wing-=cyl(pivot-[0,0,2],1.15,13)
        parts["soft_"+side]=wing.translate(-pivot).rotate([0,0,sign*POSES[pose]]).translate(pivot)
    for name,solid in parts.items():
        if solid.volume()<=0 or len(solid.decompose())!=1:
            raise ValueError(f"Disconnected {name}")
    return parts


def seated(solid):
    return solid.transform([[0,1,0,OFFSET[1]],[0,0,1,OFFSET[2]],[1,0,0,OFFSET[0]]])


def shoulder_parts(pose="closed"):
    """Return stem, soft_tip, soft_outboard, soft_inboard in seated stock STL mm.

    Fuse only stem into the cassette. M2 pivot screws and nuts are separate hardware.
    """
    return {name:seated(solid) for name,solid in trunk_parts(pose).items()}


def axial_compliance(modulus=2000., poisson=.35):
    """Small-deflection 3D circular beam estimate for a fore-aft tip load, mm/N."""
    compliance=0.
    shear=modulus/(2*(1+poisson))
    for kind,start,end,r in segments('closed'):
        if kind not in ('stem','central'):
            continue
        length=np.linalg.norm(end-start)
        axis=(end-start)/length
        front=np.array([1.,0,0])
        inertia=np.pi*r**4/4
        # Simpson integration is exact for the squared linear bending moment.
        for t,w in [(0,1),(.5,4),(1,1)]:
            m=np.cross(TOUCH-(start+t*(end-start)),front)
            torsion=m@axis
            bend=m-torsion*axis
            compliance+=length*w/6*((axis@front)**2/(modulus*np.pi*r*r)
                    + bend@bend/(modulus*inertia)+torsion*torsion/(shear*2*inertia))
    return float(compliance)


def export(folder):
    here=Path(__file__).resolve().parent
    stack=runpy.run_path(str(here/'stack/stack.py'))
    writer=stack['tail']['write_stl']
    part=shoulder_parts('closed')
    cassette=stack['cartridge']()
    union_volume=(part['stem']^cassette).volume()
    if union_volume<40:
        raise ValueError('Cassette and arm have insufficient shared volume')
    fitted=cassette+part['stem']
    # Keep the fused stem and soft centre pad in the SAME rigid registration.
    # A slicer can import them as two materials of one assembly.
    origin=fitted.rotate((90,0,0)).bounding_box()
    shift=[-origin[0],-origin[1],-origin[2]]
    folder.mkdir(exist_ok=True,parents=True)
    writer(folder/'shoulder-cartridge-stem.stl',fitted.rotate((90,0,0)).translate(shift))
    writer(folder/'shoulder-soft-centre.stl',part['soft_tip'].rotate((90,0,0)).translate(shift))
    for side in ['inboard','outboard']:
        # Trunk XY is the wing's flat bearing surface; trunk Z is its screw axis.
        lobe=trunk_parts('closed')['soft_'+side]
        lo=lobe.bounding_box()[:3]
        writer(folder/f'shoulder-soft-{side}.stl',lobe.translate(-np.array(lo)))
    materials={'stem + cartridge':1240,'soft_tip':1200,'soft_outboard':1200,'soft_inboard':1200}
    solids={'stem + cartridge':fitted,**{n:s for n,s in part.items() if n!='stem'}}
    for path in folder.glob('shoulder-*.stl'):
        raw=path.read_bytes()
        n=int.from_bytes(raw[80:84],'little')
        if len(raw)!=84+n*50 or n<4:
            raise ValueError(f'Bad STL {path}')
        tri=np.ndarray((n,3,3),dtype='<f4',buffer=raw,offset=96,strides=(50,12,4))
        if not np.isfinite(tri).all(): raise ValueError(f'Bad triangles {path}')
        verts,idx=np.unique(tri.reshape(-1,3),axis=0,return_inverse=True)
        f=idx.reshape(-1,3)
        edges=np.sort(np.concatenate([f[:,[0,1]],f[:,[1,2]],f[:,[2,0]]]),axis=1)
        if np.any(np.unique(edges,axis=0,return_counts=True)[1]!=2):
            raise ValueError(f'Non-manifold STL edges {path}')
    stats=dict(fusion_mm3=round(union_volume,3),printed_solid_mass_g={
        n:round(s.volume()*materials[n]*1e-6,3) for n,s in solids.items()},
        axial_compliance_mm_per_N_at_2GPa=round(axial_compliance(),4),
        registered_stem_print_bounds_mm=np.asarray(fitted.rotate((90,0,0)).translate(shift).bounding_box()).round(2).tolist())
    (here/'shoulder-print.json').write_text(json.dumps(stats,indent=2)+'\n',encoding='utf-8')
    print(stats)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('output',type=Path)
    export(p.parse_args().output)
