"""Play the shipped 1.7 s roulade after removing the entire necklace and tail pack."""
import argparse
import json
from pathlib import Path
import runpy

import mujoco as mj
import numpy as np
from rail_only import configure_rails


def roll(base,load,configure,settle):
    rig=base(0,0,configure=configure)
    m,d=rig.model,rig.data
    for _ in range(round(settle/.02)):
        rig.tick()
    obs=np.zeros_like(rig.obs)
    policy,home,scale,feed=load(np,obs,'xgoduck_roulade.onnx')
    filt=np.zeros(14)
    up=[]
    # Roulade runs exclusively in the runtime for 85 slots, with 0.05
    # previous-action smoothing. Stock joint limits and gravity stay active.
    for i in range(85):
        mj.mj_forward(m,d)
        obs[0,:3]=d.sensordata[rig.gyro]
        obs[0,3:6]=-d.xmat[rig.trunk,6:9]
        obs[0,6:20]=d.qpos[rig.qpos]-home
        obs[0,20:34]=d.qvel[rig.qvel]
        act=policy.session.run(None,feed)[0][0]
        obs[0,34:48]=act
        filt=.05*filt+.95*act
        d.ctrl[rig.actuators]=home+scale*filt
        for _ in range(4):
            mj.mj_step(m,d)
            up.append(d.xmat[rig.trunk,6:9].copy())
    up=np.array(up)
    # Identify the somersault plane from the actual orientation trace and
    # measure the continuous turn, including passage through upside down.
    _,_,basis=np.linalg.svd(up[:,:2],full_matrices=False)
    plane=np.unwrap(np.arctan2(up[:,:2]@basis[0],up[:,2]))
    rotation=abs(np.rad2deg(plane[-1]-plane[0]))
    for _ in range(125):
        rig.tick()
    vertical=float(d.xmat[rig.trunk,8])
    return dict(settle_s=settle,robot_mass_g=round(1000*float(m.body_mass.sum()),3),
                turn_deg=round(rotation,1),min_up_dot=round(float(up[:,2].min()),3),
                end_roll_up_dot=round(float(up[-1,2]),3),end_walk_up_dot=round(vertical,3),
                subsequent_mode=rig.recovery.phase,subsequent_transitions=rig.recovery.transitions,
                pass_roll=bool(rotation>=300 and up[:,2].min()<-.75 and up[-1,2]>.75 and
                               vertical>.9 and rig.recovery.transitions==0))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('runtime',type=Path)
    p.add_argument('output',type=Path)
    args=p.parse_args()
    c=runpy.run_path(str(args.runtime/'tools/check-duck-hardware.py'))
    from walk_sim import load_sim_policy
    folder=Path(__file__).resolve().parents[1]/'v5-slide-on-backpack'/'removable-rails'
    receiver=runpy.run_path(str(folder/'receiver_physics.py'))['configure_receiver']
    result={name:[roll(c['WalkSimulation'],load_sim_policy,configure,t) for t in (.2,.5,1.,2.)]
            for name,configure in [('stock',None),('both_removed_rails_remain',configure_rails),
                                   ('both_removed_flush_receiver',receiver)]}
    args.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
