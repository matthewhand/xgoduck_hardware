"""Show the co-printed jaws next to the corresponding unmodified stock geometry."""
from pathlib import Path
import runpy
import sys

import numpy as np
from PIL import Image, ImageDraw
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parents[1]))
from cad_figure import render,label,bg

p=runpy.run_path(str(HERE/'coprint.py'))
parts={s:p['build'](s) for s in p['STOCK']}
cad=p['CAD'];jaw=p['FIT']['jaw_pose']
rigid=[119,151,151];rigid_lower=[136,152,115];tpu=[238,168,78];pad=[82,99,91]


def mesh(s,c):
    m=s.to_mesh()
    return (m.vert_properties[m.tri_verts,:3].copy(),c)


def jaw_scene(stock):
    top,bottom=parts.values();pitch=14
    top_pad=cad['read_stock']('MOUTH_UP_TPU').rotate((180,0,0)).translate((0,-43.840,11))
    lower_pad=cad['read_stock']('MOUTH_BOTTOM_TPU').translate((0,-15.54,2.8))
    body='stock' if stock else 'rigid'
    result=[mesh(top[body],rigid),mesh(jaw(bottom[body],pitch),rigid_lower),
            mesh(top_pad,pad),mesh(jaw(lower_pad,pitch),pad)]
    if not stock:
        result.extend([mesh(top['soft'],tpu),mesh(jaw(bottom['soft'],pitch),tpu)])
    return result


def cross_section():
    r=parts['upper'];yy=cad['FRONT']['upper']+p['STATIONS']['upper'][2]
    canvas=Image.new('RGB',(430,166),bg);draw=ImageDraw.Draw(canvas)
    def signed(a):return np.sum(a[:,0]*np.roll(a[:,1],-1)-a[:,1]*np.roll(a[:,0],-1))
    for solid,color in [(r['rigid'],rigid),(r['teeth'][4],tpu)]:
        for poly in sorted(solid.slice(12.62).to_polygons(),key=lambda a:-abs(signed(a))):
            xy=[(round((x-40)*44+52),round(83-(y-yy)*32)) for x,y in poly]
            draw.polygon(xy,fill=tuple(color) if signed(poly)>0 else bg)
    return canvas


def main():
    im=Image.new('RGB',(1140,832),bg)
    label(im,(26,12),'Co-printed TPU cheek teeth',32,True)
    label(im,(26,63),'Four individual teeth per cheek, above and below. Each flexible root is keyed inside the rigid wall.',18)
    for x,stock in [(0,False),(570,True)]:
        img,_=render(jaw_scene(stock),568,287,[1,-1.25,.29],[0,0,1],focus=[0,-8,10],k=3.56)
        im.paste(img,(x,105))
    label(im,(30,385),'Keyed TPU teeth',23,True)
    label(im,(596,385),'Smooth stock shells',23,True)
    close,_=render(jaw_scene(False),540,220,[1,-.20,.06],[0,0,1],focus=[44,-37,5],k=9.1)
    im.paste(close,(22,438))
    im.paste(cross_section(),(612,449))
    label(im,(33,674),'Side view of the operating gap',21,True)
    label(im,(33,714),'Upper and lower tips pass with 0.72 mm clearance in',17)
    label(im,(33,741),'the sampled 0–30° jaw sweep.',17)
    label(im,(610,674),'Horizontal cut through one root',21,True)
    label(im,(610,714),'The 4.15 mm TPU crossbar sits behind a 1.80 mm neck.',17)
    label(im,(610,741),'The rigid print closes around both shoulders.',17)
    im.save(HERE/'figures'/'coprint-teeth.png',optimize=True)


if __name__=='__main__':
    main()
