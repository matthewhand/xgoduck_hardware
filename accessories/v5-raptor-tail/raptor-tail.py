"""Top-entry V5 V1.1 tail, in stock enclosure millimetres. +Y is up, -Z aft."""
import argparse
import json
from functools import lru_cache
from pathlib import Path
import runpy

import numpy as np
from manifold3d import Manifold, OpType

HERE=Path(__file__).resolve().parent
BACKPACK=HERE.parent/'v5-slide-on-backpack'
CAD=runpy.run_path(str(BACKPACK/'charging-pack.py'))
block,checked,write_stl = (CAD[k] for k in ['block','checked','write_stl'])
ROOT=np.array([0.,25.,-14.])
KNEE=np.array([0.,29.,-54.])
KNEE_FRAME=np.array([[0.,-1,0],[1,0,0],[0,0,1]])
ANGLE=16.0
CREST=6.6
BOLTS=[[-34.5,26.5,-3.4],[34.5,26.5,-3.4]]
JOINTS=[dict(child='proximal',parent='mount',axis=[1,0,0],origin_mm=ROOT.tolist(),range_deg=[-ANGLE,ANGLE]),
        dict(child='distal',parent='proximal',axis=[0,1,0],origin_mm=KNEE.tolist(),range_deg=[-ANGLE,ANGLE])]


def join(items):
    return Manifold.batch_boolean(list(items),OpType.Add)


def egg(p,r):
    return Manifold.sphere(1,20).scale(r).translate(p)


def rodx(x0,x1,r,y=0,z=0):
    return Manifold.cylinder(x1-x0,r,r,32).rotate((0,90,0)).translate((x0,y,z))


def position(s,origin,matrix=None):
    if matrix is None: return s.translate(origin)
    return s.transform(np.column_stack([matrix,origin]))


def arc(value):
    theta=np.deg2rad(value)
    return CREST*np.cos(theta), CREST*np.sin(theta)


def fork():
    # Smaller left ear and shrouded right stop ear, joined by a rear crossbar.
    parts=[]
    for lo,hi,r in [(-7,-5,4.9),(5,7,8.0)]:
        ear=rodx(lo,hi,r)
        heel=block(lo,hi,-3.4,3.4,6.1,8.55)
        parts.append(Manifold.batch_hull([ear,heel]))
    parts.append(block(-7,7,-3.4,3.4,6.3,8.55))
    # The radial tongue touches each integral stop outside the simulation range.
    for theta in [-37.8,37.8]:
        y,z=arc(theta)
        parts.append(rodx(3.4,5.3,1.12,y,z))
    return join(parts)-rodx(-9,9,1.64)


def knuckle():
    hub=rodx(-4.55,4.55,4.2)
    tongue=Manifold.batch_hull([rodx(3.1,4.65,1.10,3.2,0),rodx(3.1,4.65,1.10,CREST,0)])
    return (hub+tongue)-rodx(-7,7,1.64)


def shaft():
    # The TPU retainer grips the last 2 mm of the pin. Keep the heads outside
    # the fixed clevis so axial squeeze cannot bind the moving knuckle.
    shaft=rodx(-8.4,9.8,1.46)+rodx(-8.4,-7.25,2.75)
    cap=rodx(7.25,9.8,2.65)-rodx(7.15,9.9,1.34)
    return shaft,cap


def tapered(points):
    sections=[egg(p,r) for p,r in points]
    return join(Manifold.batch_hull([a,b]) for a,b in zip(sections,sections[1:]))


def metal():
    hardware={}
    for idx,(x,y,_) in enumerate(BOLTS):
        shank=Manifold.cylinder(8,1.49,1.49,24).translate((x,y,-6.2))
        head=Manifold.cylinder(2.605,2.85,2.85,24).translate((x,y,-8.8))
        nut=Manifold.cylinder(2.3,3.175,3.175,6).translate((x,y,-.99))
        hardware[f'yoke-screw-{idx+1}']=join([shank,head,nut])
    return hardware


def pocket():
    return runpy.run_path(str(BACKPACK/'pack-id.py'))['tag_pocket']()


@lru_cache(maxsize=6)
def base(clearance):
    stock=CAD['stock_enclosure']()
    fixed,pack,rigid,tooth=CAD['build'](stock,clearance)
    # Close both contact pockets on this unpowered cover and make headward ears
    # wide enough for an M3 clamp; their nuts clear the stock enclosure sides.
    head=block(-38,38,23.45,33.1,-3.2,-1.0)
    holes=join(Manifold.cylinder(12,1.68,1.68,36).translate((x,y,-9)) for x,y,_ in BOLTS)
    cover=((pack+head)-holes)-pocket()
    released=((rigid+head)-holes)-pocket()+tooth.translate((0,0,-.85))
    plate=block(-38,38,23.3,29.7,-6.2,-3.2)
    # The root yoke is a separate print, assembled after the cover is printed.
    # Its flat face seats on the cover; two outboard screws spread the tail load.
    yoke=(plate+position(fork(),ROOT))-holes
    return stock,fixed,cover,released,yoke


def components(variant='standard',clearance=.25):
    stock,fixed,cover,_,yoke=base(clearance)
    body=tapered([([0,25,-16.3],[3.05,2.8,2.1]),([0,25.1,-22.8],[3.25,2.6,2.3]),
                  ([0,26.4,-29],[5.5,4.1,2.7]),([0,27.1,-34],[5.0,3.9,3.7]),([0,29,-46.8],[3.6,3.2,2.1])])
    root=position(knuckle(),ROOT)
    outer=position(fork(),KNEE,KNEE_FRAME)
    proximal=join([root,body,outer])-position(rodx(-9,9,1.64),ROOT)
    end=40 if variant=='standard' else 25
    distal_shape=tapered([([0,29,-56.4],[3.1,3.2,2.2]),([0,30.2,-62],[4.25,3.5,3.0]),
                          ([0,32.6,-54-end*.60],[2.9,2.6,2.6]),([0,35.7,-54-end],[1.5,1.6,2.0])])
    distal=(position(knuckle(),KNEE,KNEE_FRAME)+distal_shape)-position(rodx(-9,9,1.64),KNEE,KNEE_FRAME)
    pin,cap=shaft()
    return {'rail-addition':('mount',fixed-stock),'tail-cover':('mount',cover),'root-yoke':('mount',yoke),
            'root-pin':('mount',position(pin,ROOT)),'root-cap':('mount',position(cap,ROOT)),
            **{name:('mount',part) for name,part in metal().items()},
            'proximal-link':('proximal',proximal),'distal-pin':('proximal',position(pin,KNEE,KNEE_FRAME)),
            'distal-cap':('proximal',position(cap,KNEE,KNEE_FRAME)),'distal-link':('distal',distal)}


def assembly_parts(variant='standard',clearance=.25):
    parts=components(variant,clearance)
    return {group:join(s for g,s in parts.values() if g==group) for group in ('mount','proximal','distal')}


def rotate(s,point,angles):
    return s.translate(-point).rotate(angles).translate(point)


def moments(s):
    mesh=s.to_mesh()
    a,b,c=mesh.vert_properties[mesh.tri_verts,:3].astype(float).transpose(1,0,2)
    v=np.einsum('ij,ij->i',a,np.cross(b,c))/6
    return (np.einsum('i,ij->j',v,a+b+c)/(4*sum(v))).tolist()


def part_mass(name,s):
    # M3 hardware allowance is per bolt+nut. Tag is handled as a separate
    # installed optional part once its stock and adhesive are known.
    if name.startswith('yoke-screw'): return dict(low=.75,nominal=1.02,high=1.35)
    densities=(.98,1.2,1.30) if name.endswith('cap') else (.90,1.24,1.33)
    return dict(zip(('low','nominal','high'),(s.volume()*d/1000 for d in densities)))


def summary(parts):
    props={name:dict(group=g,volume_cm3=round(s.volume()/1000,6),
                     center_mm=np.round(moments(s),5).tolist(),mass_g=part_mass(name,s)) for name,(g,s) in parts.items()}
    def aggregate(records):
        totals={k:round(sum(rec['mass_g'][k] for rec in records),5) for k in ('low','nominal','high')}
        centroid=np.average([rec['center_mm'] for rec in records],weights=[rec['mass_g']['nominal'] for rec in records],axis=0)
        return dict(mass_g=totals,centroid_mm=centroid.round(5).tolist())
    return dict(parts=props,groups={g:aggregate([r for r in props.values() if r['group']==g]) for g in ('mount','proximal','distal')},
                with_printed=aggregate(list(props.values())),scope='Increment relative to stock including net enclosure rails, yoke, screws and both moving links. Excludes optional NFC tag.')


def drawing(parts):
    out=[]
    for name,(group,s) in parts.items():
        if name=='rail-addition': continue # Duck Lab already displays the same fixed rails.
        m=s.simplify(.025).to_mesh();p=m.vert_properties[:,:3]
        out.append(dict(name=name,group=group,positions_mm=p.round(4).ravel().tolist(),indices=m.tri_verts.ravel().tolist()))
    return out


def bed(s,orientation):
    p=s.rotate(orientation)
    lo,hi=np.array(p.bounding_box()).reshape(2,3)
    return p.translate((-(lo[0]+hi[0])/2,-(lo[1]+hi[1])/2,-lo[2]))


def export(output,runtime):
    output.mkdir(exist_ok=True,parents=True)
    parts=components()
    axes={'tail-cover':(0,0,0),'root-yoke':(180,0,0),'proximal-link':(0,90,0),
          'distal-link':(90,0,0),'root-pin':(0,-90,0),'root-cap':(0,-90,0)}
    for name,orientation in axes.items():
        item=parts[name][1]
        checked(item)
        write_stl(output/(name+'.stl'),bed(item.simplify(.007),orientation))
    short=components('short')
    write_stl(output/'distal-short.stl',bed(short['distal-link'][1].simplify(.007),axes['distal-link']))
    layout=dict(version=1,frame='V5 enclosure STL millimetres, +Y toward head, -Z aft',
                joints=JOINTS,estimate=summary(parts),short_estimate=summary(short))
    (HERE/'tail-layout.json').write_text(json.dumps(layout,indent=2)+'\n',encoding='utf-8')
    if runtime:
        mesh=dict(**layout,meshes=drawing(parts))
        (runtime/'assets/sim/v5-raptor-tail.json').write_text(json.dumps(mesh,separators=(',',':'))+'\n',encoding='utf-8')
    print(layout['estimate']['with_printed'])


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('output',type=Path)
    ap.add_argument('--runtime',type=Path)
    args=ap.parse_args()
    export(args.output,args.runtime)
