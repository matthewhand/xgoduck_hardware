"""TPU open-backed necklace and its two three-fingered T. rex paws."""
from pathlib import Path
import json
import runpy
import sys
import numpy as np
from PIL import Image
from manifold3d import Manifold

HERE=Path(__file__).resolve().parent
TAIL=HERE.parent
sys.path.insert(0,str(TAIL.parent))
from cad_figure import render,label,bg
cad=runpy.run_path(str(TAIL.parent/'v5-dino-beak/dino-beak.py'))
digits=runpy.run_path(str(HERE/'digits.py'))['digits']


def ball(p,r):
    return Manifold.sphere(1,24).scale((r,r,r) if np.isscalar(r) else r).translate(p)


def link(p,q,r,s):
    return Manifold.batch_hull([ball(p,r),ball(q,s)])


def build():
    ang=np.deg2rad(np.linspace(-148,148,19))
    arc=np.column_stack((16*np.cos(ang),20*np.sin(ang),np.zeros(len(ang))))
    collar=sum([link(p,q,2.2,2.2) for p,q in zip(arc[:-1],arc[1:])],Manifold())
    arms=Manifold()
    claws=Manifold()
    for sign in (-1,1):
        root=np.array([8.,sign*18,0])
        collar+=ball(root,[3.5,3,3.5])
        elbow=np.array([11,sign*5,-21.])
        palm=np.array([28,sign*4,-14.])
        limb=link((0,0,0),elbow,2.15,3.1)+link(elbow,palm,3.1,2.6)+ball(palm,[4.5,3.5,2.7])
        arms+=limb.translate(root)
        for digit in digits(sign):
            finger=sum([link(p,q,r,s) for (p,r),(q,s) in zip(digit[:-1],digit[1:])],Manifold())
            assert (limb^finger).volume()>3
            claws+=finger.translate(root)
    return collar,arms,claws


def main():
    collar,arms,claws=build()
    entire=collar+arms+claws
    cad['check_solid'](entire)
    # Lay one side toward the bed. The elbow and collar overhangs need slicer support.
    bed=entire.rotate((90,0,0))
    path=HERE/'three-claw-necklace.stl'
    cad['write_stl'](bed.translate((0,0,-bed.bounding_box()[2])),path)
    runpy.run_path(str(TAIL.parent/'v5-dino-beak/verify-fit.py'))['check_export'](path)
    m=entire.volume()*.00120
    # Each distal cross-section should contain three separate finger tips.
    for sign in (-1,1):
        cut=claws ^ Manifold.cube((6,21,20)).translate((43.7,sign*22-10.5,-27))
        assert len([p for p in cut.decompose() if p.volume()>0])==3
    raw=lambda p: p.to_mesh().vert_properties[p.to_mesh().tri_verts,:3].astype(float)
    view=[(raw(collar),[77,152,143]),(raw(arms),[228,150,72]),(raw(claws),[239,201,145])]
    sheet=Image.new('RGB',(1000,430),bg)
    for i,(eye,target,scale,title) in enumerate([([1,.52,.70],None,None,'Removable TPU necklace'),
                                                ([.84,-.49,.83],[41,-22,-14],16,'Three rounded claws per paw')]):
        im,_=render(view,500,345,eye,[0,0,1],focus=target,k=scale)
        sheet.paste(im,(i*500,64))
        label(sheet,(i*500+16,15),title,23,True)
    sheet.save(HERE/'three-claw-necklace.png',optimize=True)
    frames=[]
    for yaw in np.linspace(-.78,-.78+2*np.pi,48,endpoint=False):
        eye=[np.cos(yaw),np.sin(yaw),.45]
        frame,_=render(view,500,360,eye,[0,0,1],focus=[14,0,-9],k=5.45)
        frames.append(frame)
    frames[0].save(HERE/'three-claw-turntable.webp',save_all=True,append_images=frames[1:],
                   duration=90,loop=0,quality=78,method=4)
    (HERE/'print-fit.json').write_text(json.dumps(dict(fingers_per_paw=3,tip_radius_mm=1.12,
       print_mass_g_at_1p20=round(m,3),added_finger_mass_g=round((entire-(collar+arms)).volume()*.0012,3)),indent=2)+'\n')

if __name__=='__main__':main()
