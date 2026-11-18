"""Loop the registered upper and lower V5 prints through an illustrative opening."""
import argparse
from pathlib import Path
import runpy
import sys
import numpy as np
from PIL import Image, ImageDraw
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent))
from cad_figure import render, label, ink, muted, bg
preview=runpy.run_path(str(HERE/'export-preview.py'))
PALETTE={
    "head-shell":[117,149,136],"head-hardware":[78,90,92],
    "head-bottom":[103,157,139],"lower-jaw":[89,141,121],
    "upper-pad":[77,91,91],"lower-pad":[77,91,91],
    "upper-teeth":[246,220,156],"lower-teeth":[247,232,190],
    "upper-teeth-tusks":[244,217,152],"upper-tusks":[234,187,109],
}


def head(parts, degrees, upper):
    theta=np.deg2rad(-degrees)
    rotation=np.array([[1,0,0],[0,np.cos(theta),-np.sin(theta)],[0,np.sin(theta),np.cos(theta)]])
    pivot=np.array(preview["HINGE"])
    for name,group,variant,points,indices in parts:
        if variant!="all" and name not in ("lower-teeth",upper): continue
        if group=="moving": points=(points-pivot)@rotation.T+pivot
        yield points[indices].astype(np.float32),PALETTE[name]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runtime",type=Path)
    parser.add_argument("--output",type=Path,default=HERE/"figures/dino-chomp.gif")
    args=parser.parse_args()
    shapes=list(preview['meshes'](args.runtime))
    frames=[];times=[]
    for i in range(36):
        angle=13*(1-np.cos(2*np.pi*i/36))
        im=Image.new('RGB',(1060,509),bg)
        label(im,(24,11),'Dinosaur tooth cuffs',27,True)
        label(im,(558,11),'With the twin tapping tusks',26,True)
        label(im,(25,53),'Front three-quarter view',16,False,muted)
        label(im,(558,53),'Side view',16,False,muted)
        cuts=[('upper-teeth',[-.71,.24,-1.0],[0,-2,-25],3.43),
              ('upper-teeth-tusks',[-1,.10,-.34],[0,-2,-42],2.65)]
        for j,(upper,eye,focus,scale) in enumerate(cuts):
            panel,project=render(list(head(shapes,angle,upper)),530,351,eye,[0,1,0],focus=focus,k=scale)
            im.paste(panel,(530*j,85))
        draw=ImageDraw.Draw(im);draw.line([(530,19),(530,429)],fill='#c5d0ca')
        label(im,(25,443),f'Jaw opening   {angle:02.0f}°',18,True)
        label(im,(25,476),'V5 rigid jaw and TPU cuffs · illustrative 0–26° motion about the servo-horn axis',16,False,muted)
        frames.append(im)
        times.append(450 if i in (0,18) else 80)
    sample=Image.new('RGB',(265,128*4),bg)
    for j,i in enumerate([0,6,12,18]):sample.paste(frames[i].resize((265,128)),(0,j*128))
    palette=sample.quantize(colors=224)
    indexed=[im.quantize(palette=palette,dither=Image.Dither.NONE) for im in frames]
    args.output.parent.mkdir(exist_ok=True,parents=True)
    indexed[0].save(args.output,save_all=True,append_images=indexed[1:],duration=times,loop=0,optimize=True,disposal=1)
    print(args.output)


if __name__=='__main__':
    main()
