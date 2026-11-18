"""Render the assembled V5 meshes and a section of the shared six fasteners."""
from pathlib import Path
import runpy
import sys
import numpy as np
from PIL import Image

HERE=Path(__file__).resolve().parent
d=runpy.run_path(str(HERE/'face-plates.py'))
preview=runpy.run_path(str(d['BEAK']/'export-preview.py'))
sys.path.insert(0,str(HERE.parent))
from cad_figure import render, label
from manifold3d import Manifold

def mesh(p):
    m=p.to_mesh()
    return m.vert_properties[m.tri_verts,:3].astype(float)

def main():
    face,jaw,plate,fringe,brows,under=d['build']()
    runtime=HERE.parents[2]/'xgoduck_runtime_arduino'
    main_shell=[]
    for name,_,_,p,f in preview['stock_head'](runtime):
        pts=(p-preview['OFFSET']['upper']) @ preview['PRINT_TO_HEAD']
        main_shell.append((pts[f], [137,145,147] if name=='head-shell' else [84,94,100]))
    face_in_head=face.transform(((0,-1,0,0),(0,0,1,-54.848),(-1,0,0,33.258)))
    stock=main_shell+[(mesh(face_in_head),[202,197,180]),(mesh(d['cad']['read_stock']('HEAD_BOTTOM')),[206,200,182])]
    rims=(mesh(d['front'](plate)),[192,126,60])
    bolts=(mesh(d['front'](d['metal'](d['FACE_POINTS']).translate((0,0,d['BROW_EXTENSION'])))),[83,73,64])
    def moving(a):
        pose=lambda p:d['jaw_pose'](p,a)
        return [(mesh(pose(jaw)),[187,173,156]),(mesh(pose(d['chin'](fringe))),[105,131,142]),
                (mesh(pose(d['chin'](d['metal'](d['CHIN_POINTS'])))),[83,73,64])]
    bg=(247,247,242)
    sheet=Image.new('RGB',(1050,550),bg)
    for i,(parts,eye,focus,scale,title,caption) in enumerate([
       (stock+[rims,bolts]+moving(4),(.13,-1,.10),(0,-32,19),3.90,'The four-bolt mask','The four recessed bolts sit above the open sensor area.'),
       (stock+[rims,bolts]+moving(24),(.77,-1,-.72),(0,-27,11),3.65,'The detachable chin beard','Two underside bolts keep both jaw sides free for teeth.')]):
        im,_=render(parts,525,393,eye,[0,0,1],focus,scale)
        sheet.paste(im,(i*525,66))
        label(sheet,(20+i*525,448),title,22,True)
        label(sheet,(20+i*525,491),caption,14)
    label(sheet,(20,14),'V5  /  Masquerade face and scalloped beard',32,True)
    sheet.save(HERE/'assembled.png',optimize=True)
    # Enlarge a cutaway of the actual under-chin hardware stack.
    from manifold3d import Manifold
    cut=Manifold.cube((30,15,30)).translate((-15,0,-5))
    cup=d['boss'](0,0)
    tab=d['plate_holes'](d['cyl'](0,0,5.3,2.9,4.25),[(0,0)])
    nut=Manifold.cube((4,4,1.6)).translate((-2,-2,1.40))-d['cyl'](0,0,1,3,1.05)
    metal=d['metal']([(0,0)])
    colored=[(cup,[174,174,159]),(tab,[105,131,142]),(metal,[84,79,65]),(nut,[188,141,72])]
    section,_=render([(mesh(part-cut),color) for part,color in colored],540,285,
                     [.15,1,.18],[0,0,1],(0,0,4),25)
    guide=Image.new('RGB',(800,350),bg)
    guide.paste(section,(0,54))
    label(guide,(20,12),'Underside fastener section',26,True)
    for y,words in [(82,'Recessed button head'),(122,'Removable beard'),(191,'Side-loaded square nut'),(246,'External jaw boss')]:
        label(guide,(468,y),words,18,True)
    guide.save(HERE/'fastener-section.png',optimize=True)
if __name__=='__main__':
    main()
