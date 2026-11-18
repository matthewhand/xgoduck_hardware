"""Render assembled and stripped flush jaw hosts from their checked CAD geometry."""
from pathlib import Path
import runpy
import sys

import numpy as np
from PIL import Image, ImageDraw
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parents[1]))
from cad_figure import render,label,bg,muted
g=runpy.run_path(str(HERE/'screw-teeth.py'))
parts=g['build']()
jaw=g['FIT']['jaw_pose']
shell=[127,161,160]; lower=[140,155,119]; strip=[232,184,108]; metal=[119,134,151]; nut=[212,139,82]


def triangles(s,c):
    m=s.to_mesh()
    return (m.vert_properties[m.tri_verts,:3].copy(),c)


def scene(fitted):
    top=g['CAD']['read_stock']('MOUTH_UP_TPU').rotate((180,0,0)).translate((0,-43.840,11))
    bottom=g['CAD']['read_stock']('MOUTH_BOTTOM_TPU').translate((0,-15.54,2.8))
    bits=[triangles(top,[94,110,107]),triangles(jaw(bottom,12),[94,110,107])]
    for side,r in parts.items():
        pose=(lambda s:jaw(s,12)) if side=='lower' else (lambda s:s)
        bits.append(triangles(pose(r['mod']),shell if side=='upper' else lower))
        if fitted: bits.append(triangles(pose(r['rows']),strip))
        for i,item in enumerate(r['hardware']):
            if not fitted and i%2==0: continue
            bits.append(triangles(pose(g['pair'](item)),nut if i%2 else metal))
    return bits


def cross_section():
    r=parts['upper']
    y,z=g['sites']('upper')[0]
    items=[(r['mod'],shell),(r['strip'],strip),(r['hardware'][0],metal),(r['hardware'][1],nut)]
    canvas=Image.new('RGB',(510,165),bg)
    draw=ImageDraw.Draw(canvas)
    for shape,color in items:
        polys=shape.rotate((90,0,0)).slice(y+.012).to_polygons()
        def signed(p):return np.sum(p[:,0]*np.roll(p[:,1],-1)-p[:,1]*np.roll(p[:,0],-1))
        for poly in sorted(polys,key=lambda a:-abs(signed(a))):
            xy=[(round(39+(x-39)*32),round(85+(zz+z)*32)) for x,zz in poly]
            draw.polygon(xy,fill=tuple(color) if signed(poly)>0 else bg)
    return canvas


def main():
    out=HERE/'figures';out.mkdir(exist_ok=True)
    im=Image.new('RGB',(1140,780),bg)
    label(im,(28,10),'Flush jaw mounts and removable cheek teeth',32,True)
    label(im,(28,60),'Two recessed M2 screws per four-tooth strip. The square nuts stay inside the original jaw walls.',18)
    open_scene=scene(False)
    for pos,obj in [(0,scene(True)),(570,open_scene)]:
        view,_=render(obj,570,280,[1,-1.18,.37],[0,0,1],focus=[0,-8,9],k=3.5)
        im.paste(view,(pos,101))
    label(im,(32,371),'Four strips installed',22,True)
    label(im,(600,371),'All strips and screws removed',22,True)
    close,proj=render(open_scene,544,182,[1,-.06,.03],[0,0,1],focus=[43.8,-34.2,7.4],k=10.0)
    im.paste(close,(23,446));im.paste(cross_section(),(603,452))
    label(im,(32,638),'Right cheek, stripped',22,True)
    label(im,(606,638),'Upper screw and captured nut, cutaway',22,True)
    label(im,(32,679),'The upper pair sits in the thicker return of the stock print.',17)
    label(im,(32,707),'The lower pair sits 14 mm apart in the lower sidewall.',17)
    label(im,(606,679),'Blue-green is the original shell profile. Gold lifts off',17)
    label(im,(606,707),'with the strip. The screw sits below its outer rim.',17)
    im.save(out/'screw-teeth.png',optimize=True)


if __name__=='__main__':
    main()
