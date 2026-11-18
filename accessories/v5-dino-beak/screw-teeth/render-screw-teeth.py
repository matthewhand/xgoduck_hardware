"""Render screw-retained side rows and a section through an actual M2 pocket."""
from pathlib import Path
import runpy
import sys

import numpy as np
from PIL import Image, ImageDraw
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parents[1]))
from cad_figure import render,label,bg,muted
g=runpy.run_path(str(HERE/'screw-teeth.py'))
p=g['build']()
jaw=g['FIT']['jaw_pose']
shell=[116,143,143]; lower=[114,135,114]; plastic=[229,187,115]; metal=[106,117,141]
nut=[192,143,87];boss=[90,151,146]


def trimesh(s,c):
    m=s.to_mesh()
    return (m.vert_properties[m.tri_verts,:3].copy(), c)


def scene(side,fitted):
    r=p[side]
    transform=(lambda s:jaw(s,11)) if side=='lower' else (lambda s:s)
    bits=[trimesh(transform(r['stock']),shell if side=='upper' else lower),trimesh(transform(r['bosses']),boss)]
    for sign in [-1,1]:
        strip=r['strip'] if sign==1 else r['strip'].mirror((1,0,0))
        if not fitted:strip=strip.translate((sign*10,0,0))
        bits.append(trimesh(transform(strip),plastic))
        for i,item in enumerate(r['hardware']):
            part=item if sign==1 else item.mirror((1,0,0))
            if not fitted and i%2==0:part=part.translate((sign*18,0,0))
            bits.append(trimesh(transform(part),metal if i%2==0 else nut))
    return bits


def cross_section(im):
    r=p['upper']
    y=g['FRONT']['upper']+4.61
    # Rotating +90 about X puts print-Y along the section axis, and Z along -v.
    parts=[(r['stock'],shell),(r['bosses'],boss),(r['strip'],plastic)]
    parts+=[(part,metal if i%2==0 else nut) for i,part in enumerate(r['hardware'][:2])]
    draw=ImageDraw.Draw(im)
    for shape,color in parts:
        polys=shape.rotate((90,0,0)).slice(y).to_polygons()
        for poly in sorted(polys,key=lambda a:-abs(np.sum(a[:,0]*np.roll(a[:,1],-1)-a[:,1]*np.roll(a[:,0],-1)))):
            signed=np.sum(poly[:,0]*np.roll(poly[:,1],-1)-poly[:,1]*np.roll(poly[:,0],-1))
            xy=[(round(56+(x-40)*19),round(733+(z+13.05)*19)) for x,z in poly]
            draw.polygon(xy, fill=tuple(color) if signed>0 else bg)
    # Cover everything outside the diagram, including clipped host geometry.
    draw.rectangle((0,614,28,832),fill=bg)
    draw.rectangle((350,614,1139,832),fill=bg)


def main():
    output=HERE/'figures';output.mkdir(exist_ok=True)
    im=Image.new('RGB',(1140,836),bg)
    allparts=scene('upper',True)+scene('lower',True)
    a,_=render(allparts,566,326,[1,-1.45,.35],[0,0,1],focus=[0,-25,9],k=3.6)
    b,_=render(scene('upper',False)+scene('lower',False),566,326,[.94,-1.25,.34],[0,0,1],focus=[0,-25,9],k=3.2)
    im.paste(a,(2,116));im.paste(b,(571,116))
    # Isolate the four-tooth right-side strip in its installed orientation.
    strip=p['upper']['strip']
    side,_=render([trimesh(strip,plastic)],302,125,[1,-.08,.16],[0,0,1],k=8.2)
    im.paste(side,(38,479))
    cross_section(im)
    label(im,(28,12),'Screw-retained cheek teeth',32,True)
    label(im,(28,62),'Four independent rows. Two recessed M2 screws per row. The stock mouth opening stays clear.',18)
    label(im,(35,445),'Installed on the two jaw prints',21,True)
    label(im,(604,445),'Exploded along the screw axes',21,True)
    label(im,(385,502),'One rounded four-tooth strip',23,True)
    label(im,(385,541),'Choose upper or lower, left or right. There is no bridge across the bill.',18)
    label(im,(35,625),'Section through one upper mount',21,True)
    label(im,(385,662),'Rigid host + print-in M2 nut',22,True)
    label(im,(385,700),'The boss projects outward from the stock cheek. The nut stays captured',18)
    label(im,(385,728),'when the strip is removed. The screw head sits 0.5 mm below the rim.',18)
    label(im,(385,781),'Grey-blue screw · tan nut · gold strip · teal external boss',16,False,muted)
    im.save(output/'screw-teeth.png',optimize=True)


if __name__=='__main__':
    main()
