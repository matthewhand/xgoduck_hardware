"""Render the seated shoulder boom from its CAD and its nominal pinhole view."""
import argparse
import json
from pathlib import Path
import runpy
import sys

import mujoco as mj
import numpy as np
from PIL import Image,ImageDraw,ImageFont

HERE=Path(__file__).resolve().parent
CHECK=runpy.run_path(str(HERE/'check-shoulder-pointer.py'))
CAD,STACK=CHECK['CAD'],CHECK['STACK']


def font(size,bold=False):
    for name in (('segoeuib.ttf','DejaVuSans-Bold.ttf') if bold else ('segoeui.ttf','DejaVuSans.ttf')):
        try: return ImageFont.truetype(name,size)
        except OSError: pass
    return ImageFont.load_default()


def view(factory,pose,mode,aim):
    def build(spec):
        CHECK['add_stack'](spec,pose)
        body=spec.body('trunk_base')
        parts={'deck':STACK['deck']()}
        if pose!='removed':
            parts.update(cart=STACK['cartridge'](),slide_pin=STACK['pin'](),**CAD['shoulder_parts'](pose))
            for sign in [-1,1]:
                p=CAD['HUB']+[0,sign*5.5,0]
                screw=CAD['cyl'](p-[0,0,3.1],1,8.3)+CAD['cyl'](p+[0,0,5.05],1.85,1.25)
                parts['screw'+str(sign)]=CAD['seated'](screw)
        for n,solid in parts.items():
            mesh=solid.to_mesh()
            geom='show_'+n
            spec.add_mesh(name=geom,uservert=CHECK['trunk'](mesh.vert_properties[:,:3]).ravel(),
                          userface=np.ravel(mesh.tri_verts))
            color=([.96,.61,.16,1] if n=='soft_tip' else [.16,.84,.79,1] if n.startswith('soft_') else
                   [.94,.45,.12,1] if n=='stem' else [.53,.63,.67,1])
            body.add_geom(name=geom,type=mj.mjtGeom.mjGEOM_MESH,meshname=geom,
                          mass=0,contype=0,conaffinity=0,group=1,rgba=color)
        for g in spec.geoms:
            if g.name.startswith(('stack_','feeler_','pointer_')):
                g.group=5
            elif g.name.startswith('tail_mount_'):
                g.group=1
                g.rgba=[.41,.48,.49,1]
        if mode=='camera':
            jaw=spec.body('jaw_soft')
            jaw.add_camera(name='forward_test_lens',pos=spec.site('head_camera').pos,
                           quat=[1,0,0,0],fovy=45)
            # Unpowered test panel is only present for this rendered scene.
            center=CAD['TOUCH']*.001
            body.add_geom(name='dummy_panel',type=mj.mjtGeom.mjGEOM_BOX,pos=center+[.010,0,0],
                          size=[.0015,.050,.040],rgba=[.37,.44,.48,1],mass=0,contype=0,conaffinity=0,group=1)
            body.add_geom(name='dummy_key',type=mj.mjtGeom.mjGEOM_CYLINDER,pos=center+[.007,0,0],
                          quat=[.70710678,0,.70710678,0],size=[.009,.0015,0],rgba=[.88,.90,.83,1],
                          mass=0,contype=0,conaffinity=0,group=1)
    rig=factory(0,0,configure=build)
    for n,a in [('head_yaw',aim['head_yaw_deg']),('head_pitch',aim['head_pitch_deg'])]:
        rig.data.qpos[rig.model.joint(n).qposadr]=np.deg2rad(a)
    m,d=rig.model,rig.data
    mj.mj_forward(m,d)
    camera=mj.MjvCamera()
    options=mj.MjvOption()
    opts=([.032,-.013,.038],.44,116,-19) if mode=='side' else (
         ([.129,-.055,.080],.090,104,-62) if mode=='fan' else ([-.045,0,.005],.34,50,-27))
    center,distance,az,el=opts
    camera.lookat[:]=d.xpos[rig.trunk]+d.xmat[rig.trunk].reshape(3,3)@center
    camera.distance=distance;camera.azimuth=az;camera.elevation=el
    if mode=='camera':
        camera='forward_test_lens'
    m.vis.map.znear=.001
    with mj.Renderer(m,height=306,width=408) as renderer:
        renderer.update_scene(d,camera=camera,scene_option=options)
        return Image.fromarray(renderer.render())


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('runtime',type=Path)
    p.add_argument('output',type=Path)
    args=p.parse_args()
    checks=runpy.run_path(str(args.runtime/'tools/check-duck-hardware.py'))
    aim=json.loads((HERE/'shoulder-evaluation.json').read_text())['camera']['best']
    canvas=Image.new('RGB',(864,844),'#11191e');draw=ImageDraw.Draw(canvas)
    draw.text((16,12),'Right-shoulder feeler',font=font(30,True),fill='#f1f4ee')
    draw.text((16,53),'V5 accessory study  /  centre stylus and manually opening fan',font=font(16),fill='#acbfbf')
    panels=[('closed','side','Pointing pose','Soft centre pad leads both folded lobes.'),
            ('open','fan','Open fan','Two M2 pivots allow 38° outward per lobe.'),
            ('removed','park','Cassette removed','The upper deck stays on the backpack.'),
            ('closed','camera','Nominal camera view','Synthetic 45° vertical field of view, 18 mm dummy key.')]
    for i,(pose,mode,title,cap) in enumerate(panels):
        x=16+(i%2)*424;y=84+(i//2)*376
        canvas.paste(view(checks['WalkSimulation'],pose,mode,aim),(x,y))
        draw.text((x,y+310),title,fill='#f1f4ee',font=font(18,True))
        draw.text((x,y+336),cap,fill='#b6c7c4',font=font(14))
    canvas.save(args.output,optimize=True)


if __name__=='__main__':
    main()
