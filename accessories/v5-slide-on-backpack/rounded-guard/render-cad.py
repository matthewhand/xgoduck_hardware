"""Orthographic comparison of the installed covers on the same V1.1 receiver."""
import json
from pathlib import Path
import runpy
import sys

from PIL import Image

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parents[1]))
from cad_figure import render,label,bg

def main():
    cad=runpy.run_path(str(HERE/'rounded.py'))
    _,receiver,track,plain,*_=cad['base']()
    f=json.loads((HERE/'fit.json').read_text())
    keys=['shoulder_depth_mm','shoulder_center_y_mm','shoulder_half_width_mm','side_flare_mm','side_center_y_mm','side_center_z_mm']
    rounded,extension=cad['rounded'](*(f[k] for k in keys))
    panel=Image.new('RGB',(1000,414),bg)
    for i,title in enumerate(['Original guard','Rounded guard']):
        parts=[(receiver,[189,187,172]),(track,[84,101,104]),(plain,[83,148,137])]
        if i:
            parts.append((extension-plain,[218,151,68]))
        im,_=render([(cad['triangles'](part),color) for part,color in parts],500,338,
                    [.59,.32,-1],[0,1,0],focus=(0,0,4),k=4.0)
        panel.paste(im,(500*i,56))
        label(panel,(500*i+16,10),title,25,True)
    panel.save(HERE/'before-after.png',optimize=True)

if __name__=='__main__':main()
