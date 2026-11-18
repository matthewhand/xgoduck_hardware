"""Render the bare Duck and a measured sizing trial side by side for Duck Lab."""
import argparse
import runpy
from pathlib import Path

import mujoco as mj
import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE=Path(__file__).resolve().parent
study=runpy.run_path(str(HERE/'size-study.py'))


def replay(sim_type, output, case):
    rigs=[sim_type(.2,0),sim_type(.2,0,configure=lambda s:study['build'](s,132,1,1.))]
    camera=mj.MjvCamera()
    camera.distance=.54
    camera.azimuth=104
    camera.elevation=-11
    options=mj.MjvOption()
    for rig in rigs:
        if case=='right':
            mj.mju_axisAngle2Quat(rig.data.qpos[rig.root+3:rig.root+7],np.array([1.,0,0]),np.pi/2)
        for g in range(rig.model.ngeom):
            name=rig.model.geom(g).name or ''
            if name.startswith('tail_mount_'):
                # Show the registered printed mount through its collision mesh.
                rig.model.geom_group[g]=1
                rig.model.geom_rgba[g]=[.38,.46,.43,1]
    renderers=[]
    frames=[]
    try:
        renderers=[mj.Renderer(r.model,height=264,width=396) for r in rigs]
        font=ImageFont.truetype('segoeui.ttf',17) if Path('C:/Windows/Fonts/segoeui.ttf').exists() else ImageFont.load_default()
        for slot in range(400):
            for rig in rigs:
                if case=='walk':
                    rig.obs[0,51]=study['LEAN']*np.clip((rig.data.time-2)/1.5,0,1)
                rig.tick()
            if slot%5:
                continue
            frame=Image.new('RGB',(796,298),'#10181d')
            draw=ImageDraw.Draw(frame)
            for i,(rig,renderer) in enumerate(zip(rigs,renderers)):
                mj.mj_forward(rig.model,rig.data)
                camera.lookat[:]=[*rig.data.xpos[rig.trunk,:2],.125]
                renderer.update_scene(rig.data,camera=camera,scene_option=options)
                frame.paste(Image.fromarray(renderer.render()),(400*i,0))
                title=('Bare Duck','132 mm · one passive hinge')[i]
                draw.text((400*i+9,270),title,fill='#f4faf7',font=font)
            draw.text((723,270),f'{rigs[0].data.time:.1f} s',fill='#b9d6ca',font=font)
            frames.append(frame)
    finally:
        for renderer in renderers:
            renderer.close()
    frames[0].save(output,format='WEBP',save_all=True,append_images=frames[1:],
                   duration=100,loop=0,quality=75,method=3)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runtime',type=Path)
    args=parser.parse_args()
    checks=runpy.run_path(str(args.runtime/'tools/check-duck-hardware.py'))
    for case in ('walk','right'):
        replay(checks['WalkSimulation'],args.runtime/f'assets/sim/tail-size-{case}.webp',case)
