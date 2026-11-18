"""Interchangeable rounded V5 beak tools. Assembly coordinates are HEAD_BOTTOM STL mm."""
import argparse
import json
from pathlib import Path
import runpy
import numpy as np
from manifold3d import Manifold,CrossSection,JoinType,OpType

HERE=Path(__file__).resolve().parent
DINO=runpy.run_path(str(HERE.parent/'v5-dino-beak/dino-beak.py'))
UPPER_HINGE=np.array([.00325,24.05-24.89448,20.30-.50])
JAW_SHIFT=(.00325,-24.89448,-.50)
GRASP=4.5
OPEN=26.
TIPS=('pointer','fork','horn')
TIPSITES={'pointer':[[0,-120,13.2]],'fork':[[-11.5,-118,14.2],[11.5,-118,14.2]],'horn':[[7.0,-122,17.2]]}
RADII={'pointer':3.,'fork':2.7,'horn':3.2}
SEATS=(-82.,-66.)


def box(x0,x1,y0,y1,z0,z1):
    return Manifold.cube((x1-x0,y1-y0,z1-z0)).translate((x0,y0,z0))


def egg(p,r):
    return Manifold.sphere(1,28).scale(r).translate(p)


def link(a,b):
    return Manifold.batch_hull([a,b])


def union(seq):
    return Manifold.batch_boolean(seq,OpType.Add)


def cylinder_x(a,b,r,y,z):
    return Manifold.cylinder(b-a,r,r,40).rotate((0,90,0)).translate((a,y,z))


def upper_and_lower():
    rigid=DINO['read_stock']('HEAD_BOTTOM')
    pad=DINO['read_stock']('MOUTH_UP_TPU').rotate((180,0,0)).translate((0,-43.840,11))
    jaw=DINO['read_stock']('JAW').translate(JAW_SHIFT)
    lower=DINO['read_stock']('MOUTH_BOTTOM_TPU').translate((0,-15.54,2.8)).translate(JAW_SHIFT)
    return rigid,pad,jaw,lower


def open_jaw(s,deg):
    return s.translate(-UPPER_HINGE).rotate((deg,0,0)).translate(UPPER_HINGE)


def prism(poly,y0,y1,clearance=0):
    sec=CrossSection([poly])
    if clearance:sec=sec.offset(clearance,JoinType.Miter)
    return sec.extrude(y1-y0).rotate((90,0,0)).translate((0,y1,0))


# Cross-section of the common keyed tongue in X,Z viewed from the tip.
KEY=[(-4.1,4.45),(4.1,4.45),(4.1,7.75),(2.55,9.30),(-4.1,9.30)]


def handle():
    # Top aligns with upper TPU. The underside follows the opposing lower TPU
    # at a 4.5-degree opening. Reduce thickness under the rolled stock lip.
    def section(width,y,bottom,top):
        return box(-width,width,y-.42,y+.42,bottom,top)
    sections=[section(9.0,-43.8,5.69,9.80),section(11.,-46,5.56,9.80),
              section(11.,-51.8,5.05,9.80),section(8.2,-54.8,4.95,8.30),
              section(5.5,-59.3,5.15,8.20),section(5.5,-66.7,5.,8.65)]
    paddle=union([link(a,b) for a,b in zip(sections,sections[1:])])
    # Outer housing fronts at -83. A solid rear neck carries the crossbar.
    outer=box(-7.1,7.1,-83,-64.5,2.7,11.2)
    cavity=prism(KEY,-83.05,-68.65,.25)
    bars=[cylinder_x(a,b,1.75,y,6.65) for y in SEATS for a,b in [(-11.7,-6.5),(6.5,11.7)]]
    pinhole=cylinder_x(-14,14,1.40,-75,6.65)
    return (paddle+outer+union(bars))-cavity-pinhole


def retainer():
    # Rigid headed 2.5 mm pin, retained by a separate TPU collar.
    pin=cylinder_x(-9.1,9.6,1.25,-75,6.65)+cylinder_x(-9.15,-7.26,2.05,-75,6.65)
    cap=cylinder_x(7.28,9.6,2.03,-75,6.65)-cylinder_x(7.15,9.7,1.13,-75,6.65)
    return pin,cap


def tip(kind):
    shank=prism(KEY,-83.15,-69.0)
    lip=box(-5.65,5.65,-85.3,-83,3.45,10.2)
    pieces=[shank,lip]
    def e(x,y,z,r):return egg((x,y,z),r)
    begin=e(0,-84.4,6.85,(4.8,1.65,2.65))
    if kind=='pointer':
        path=[begin,e(0,-91,7.1,(3.5,3.3,2.8)),e(0,-110,11.2,(2.75,3.2,2.5)),e(*TIPSITES[kind][0],(3,3,3))]
        pieces+=[link(a,b) for a,b in zip(path,path[1:])]
    elif kind=='fork':
        for sign in [-1,1]:
            path=[begin,e(sign*5.1,-91,8.0,(3.,3.,2.65)),e(sign*10.2,-105,11.4,(2.65,3,2.4)),
                  e(sign*11.5,-118,14.2,(2.7,2.7,2.7))]
            pieces+=[link(a,b) for a,b in zip(path,path[1:])]
    elif kind=='horn':
        path=[begin,e(0,-91,11.2,(3.8,3.4,3.1)),e(0,-101,19.5,(3.35,3.6,3.1)),
              e(.65,-111,22,(3.10,3.1,3.)),e(*TIPSITES[kind][0],(3.2,3.2,3.2))]
        pieces+=[link(a,b) for a,b in zip(path,path[1:])]
    else:raise ValueError('Unknown tip')
    return union(pieces)-cylinder_x(-10,10,1.40,-75,6.65)


def stand():
    base=box(-24,24,-99,-62,-38,-34.5)
    for x in [-18.5,18.5]:
        base-=Manifold.cylinder(6,2.2,2.2,32).translate((x,-91,-39))
    for sign in [-1,1]:
        x=sign*10.05
        mast=link(box(x-3.2,x+3.2,-87,-62.5,-35,-32),
                  box(x-1.9,x+1.9,-86,-62.6,4.3,8.2))
        for y in SEATS:
            mast-=cylinder_x(x-4,x+4,2.10,y,7.0)+box(x-4,x+4,y-2.1,y+2.1,7.,11)
        mast-=box(x-4,x+4,-77.6,-72.4,4.0,11)
        base+=mast
    return base


def mesh_moments(s):
    m=s.to_mesh();a,b,c=m.vert_properties[m.tri_verts,:3].astype(float).transpose(1,0,2)
    v=np.einsum('ij,ij->i',a,np.cross(b,c))/6
    return s.volume(),np.average((a+b+c)/4,axis=0,weights=v)


def bed(s,rotation):
    p=s.rotate(rotation)
    lo,hi=np.array(p.bounding_box()).reshape(2,3)
    return p.translate((-(lo[0]+hi[0])/2,-(lo[1]+hi[1])/2,-lo[2]))


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--out',type=Path,default=HERE/'print')
    args=ap.parse_args();args.out.mkdir(exist_ok=True,parents=True)
    pa,co=retainer()
    prints={'jaw-paddle':(handle(),(0,0,0)),'shoulder-pin':(pa,(0,-90,0)),'pin-collar-tpu':(co,(0,-90,0)),
            'pickup-stand':(stand(),(0,0,0)),**{k+'-tip-tpu':(tip(k),(0,90,0)) for k in TIPS}}
    for n,(s,rr) in prints.items():
        DINO['check_solid'](s)
        DINO['write_stl'](bed(s,rr),args.out/(n+'.stl'))
        print(n,round(s.volume()/1000,3),'cm3')


if __name__=='__main__':main()
