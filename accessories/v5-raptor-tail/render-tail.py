"""Render the printable V5 tail at its registered neutral position behind the Duck."""
import argparse
from pathlib import Path
import runpy
import sys

import numpy as np
from PIL import Image, ImageDraw
from scipy.spatial.transform import Rotation

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent))
from cad_figure import render,label,bg,muted,ink
g=runpy.run_path(str(HERE/'raptor-tail.py'))


def shape(s):
    m=s.to_mesh()
    return m.vert_properties[m.tri_verts,:3].astype(np.float32)


def walking(runtime):
    sys.path.append(str(runtime/'.cache/qualification-deps'))
    import mujoco as mj
    model=mj.MjModel.from_xml_path(str(runtime/'assets/sim/model/robot_walk.xml'))
    data=mj.MjData(model)
    mj.mj_forward(model,data)
    xyz=data.xpos[model.body('trunk_base').id]
    items=[]
    for i in range(model.ngeom):
        if model.geom_group[i]!=2: continue
        k=model.geom_dataid[i]; n=model.mesh(k).name
        vertices=model.mesh_vert[model.mesh_vertadr[k]:model.mesh_vertadr[k]+model.mesh_vertnum[k]]
        faces=model.mesh_face[model.mesh_faceadr[k]:model.mesh_faceadr[k]+model.mesh_facenum[k]]
        p=(vertices@data.geom_xmat[i].reshape(3,3).T + data.geom_xpos[i]-xyz)*1000
        pos=p[:,[1,2,0]]+[.00628,39.25512,57.33226]
        color=[192,199,186] if n in ['head_roll','trunk_base','right_hip_pitch','left_hip_pitch'] else [120,139,137]
        items.append((pos[faces].astype(np.float32),color))
    return items


def pose(p,axis,origin,deg):
    return (p-origin)@Rotation.from_rotvec(np.array(axis)*np.deg2rad(deg)).as_matrix().T+origin


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runtime',type=Path)
    args=parser.parse_args()
    fixed=g['base'](.25)[1]
    parts=g['components']()
    colors={'tail-cover':[98,163,153],'root-yoke':[74,118,126],'proximal-link':[202,155,76],'distal-link':[226,180,94]}
    parts=[(k,group,shape(s),colors.get(k,[82,91,95] if 'screw' in k else [132,116,92]))
           for k,(group,s) in parts.items() if k!='rail-addition']
    duck=walking(args.runtime)
    fixed=(shape(fixed),[98,125,128])
    out=HERE/'figures';out.mkdir(exist_ok=True)
    frames=[]; times=[]
    for i in range(32):
        theta=i*2*np.pi/32
        pitch=16*np.sin(theta);yaw=16*np.sin(2*theta)
        robot=[fixed]
        for name,group,vertices,color in parts:
            if group=='distal':vertices=pose(vertices,[0,1,0],g['KNEE'],yaw)
            if group!='mount':vertices=pose(vertices,[1,0,0],g['ROOT'],pitch)
            robot.append((vertices.astype(np.float32),color))
        im=Image.new('RGB',(1080,538),bg)
        label(im,(22,12),'Tail on the V5 Duck',25,True)
        label(im,(554,12),'Two passive hinges',25,True)
        label(im,(23,51),'Neutral walking-model pose',16,False,muted)
        label(im,(555,51),'Root pitch and tip yaw',16,False,muted)
        broad,_=render(duck+robot,531,382,[.9,.25,-.56],[0,1,0],focus=[0,43,21],k=1.26)
        im.paste(broad,(0,79))
        close,_=render(robot,540,382,[.89,.59,-.65],[0,1,0],focus=[0,11,-36],k=3.05)
        im.paste(close,(540,79))
        ImageDraw.Draw(im).line((537,8,537,469),fill='#becbc6')
        label(im,(23,482),'Standard tip',17,True)
        label(im,(555,469),f'Pitch {pitch:+.0f}°    Yaw {yaw:+.0f}°',17,True)
        label(im,(555,497),'Articulation range shown by hand posing in CAD',15,False,muted)
        frames.append(im);times.append(120 if i else 550)
    swatch=Image.new('RGB',(270,134*4),bg)
    for k,i in enumerate([0,4,8,20]):swatch.paste(frames[i].resize((270,134)),(0,k*134))
    pal=swatch.quantize(colors=192)
    pics=[im.quantize(palette=pal,dither=Image.Dither.NONE) for im in frames]
    pics[0].save(out/'tail-articulation.gif',save_all=True,append_images=pics[1:],duration=times,loop=0,optimize=True,disposal=1)
    # Explode the joints along the pin axes and separate the bolted yoke from the cover.
    im=Image.new('RGB',(1040,627),bg)
    label(im,(22,12),'Tail assembly',29,True)
    label(im,(23,56),'Two rigid links, two hinge pins and two flexible retaining caps',18,False,muted)
    apart={'tail-cover':[0,0,28],'root-yoke':[0,0,10],'root-pin':[-23,0,10],'root-cap':[24,0,10],
           'yoke-screw-1':[0,0,0],'yoke-screw-2':[0,0,0],
           'proximal-link':[0,0,-9],'distal-pin':[0,-24,-9],'distal-cap':[0,23,-9],'distal-link':[0,0,-31]}
    obj=[((t+apart[k]).astype(np.float32),co) for k,_,t,co in parts]
    panel,pr=render(obj,1040,420,[1,.57,-.52],[0,1,0],focus=[0,12,-49],k=3.6)
    im.paste(panel,(0,96))
    draw=ImageDraw.Draw(im)
    for j,(p,offset) in enumerate([([-34,27,25],[33,-25]),([4,29,-4],[34,-55]),([-29,25,-4],[-39,37]),([1,27,-40],[-10,51]),([0,57,-63],[-35,-31]),([1,33,-110],[-27,63])]):
        a=pr(p)[:2]+[0,96];b=a+offset
        draw.line((tuple(a),tuple(b)),fill='#27474a',width=2)
        draw.ellipse((b[0]-12,b[1]-12,b[0]+12,b[1]+12),fill='#253c40')
        label(im,(b[0]-5,b[1]-11),str(j+1),16,True,'white')
    label(im,(24,530),'1  Top-entry cover       2  Root yoke       3  Pitch pin       4  Proximal link       5  Yaw cap       6  Distal link',17,True)
    label(im,(25,574),'Both joints use the same pin and cap. Fit one of the two distal lengths.',17,False,muted)
    im.save(out/'tail-exploded.png',optimize=True)


if __name__=='__main__': main()
