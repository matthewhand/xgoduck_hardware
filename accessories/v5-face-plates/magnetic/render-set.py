"""Actual V5 face, pad, jaw and insert meshes with accessories off and on."""
from pathlib import Path
import runpy
import sys

from PIL import Image

HERE=Path(__file__).resolve().parent
acc=HERE.parents[1]
d=runpy.run_path(str(HERE.parent/'face-plates.py'))
prev=runpy.run_path(str(d['BEAK']/'export-preview.py'))
br=runpy.run_path(str(HERE/'brow.py'))
ch=runpy.run_path(str(HERE/'chin.py'))
teeth=runpy.run_path(str(d['BEAK']/'screw-teeth/screw-teeth.py'))
sys.path.insert(0,str(acc))
from cad_figure import render,label,bg


def triangles(s):
    m=s.to_mesh()
    return m.vert_properties[m.tri_verts,:3].astype(float)


def main():
    t=teeth['build']()
    j=ch['build'](t['lower']['mod'])
    fs=br['magnetic_face']().transform(((0,-1,0,0),(0,0,1,-54.848),(-1,0,0,33.258)))
    head=[(triangles(fs),[203,197,178]),(triangles(t['upper']['mod']),[203,197,178])]
    for name,_,_,p,f in prev['stock_head'](acc.parents[1]/'xgoduck_runtime_arduino'):
        pts=(p-prev['OFFSET']['upper']) @ prev['PRINT_TO_HEAD']
        if name=='head-shell':
            # Stock aggregate mesh includes the original rigid front surface.
            # Use this only for outer shell/hardware; actual engineered jaws are separate.
            color=[130,137,138]
        else:color=[76,88,91]
        head.append((pts[f],color))
    head.append((triangles(d['cad']['read_stock']('MOUTH_UP_TPU').rotate((180,0,0)).translate((0,-43.840,11))),[105,108,104]))
    def head_parts(fitted,angle):
        pose=lambda p:d['jaw_pose'](p,angle)
        parts=head+[(triangles(pose(j['host'])),[186,174,152]),
                    (triangles(pose(d['cad']['read_stock']('MOUTH_BOTTOM_TPU').translate((0,-15.54,2.8)))),[105,108,104])]
        if fitted:
            parts.extend([(triangles(d['front'](br['mask_local']())),[195,133,66]),
                          (triangles(pose(j['beard'])),[99,142,146])])
        return parts
    fig=Image.new('RGB',(1100,440),bg)
    for i,(worn,eye,a,title) in enumerate([(False,[.68,-1,-.4],11,'Accessories removed'),
                                           (True,[.34,-1,-.20],12,'Magnetic mask and beard fitted')]):
        im,_=render(head_parts(worn,a),550,340,eye,[0,0,1],focus=(0,-20,19),k=3.3)
        fig.paste(im,(550*i,60))
        label(fig,(550*i+16,16),title,23,True)
    label(fig,(16,395),'Stock chin and brow surfaces cover the steel targets.',16)
    label(fig,(566,395),'The removable pieces carry the magnets and spacers.',16)
    fig.save(HERE/'off-and-on.png',optimize=True)

if __name__=='__main__':main()
