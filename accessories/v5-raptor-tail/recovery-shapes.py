"""Screen printable rounded mount shoes and an unpowered pivot on the 167 mm curved tail."""
import argparse
import json
from pathlib import Path
import runpy
import sys

import mujoco as mj
import numpy as np

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
SIZE=runpy.run_path(str(HERE/'size-study.py'))
POSTURE=runpy.run_path(str(HERE/'posture-study.py'))


def shoe_centers(shoe, sign, lower_anchor_z=-63):
    """Return the two mounting roots and intervening roll apex in trunk mm."""
    dx,flare,height,_=shoe
    return np.array([[-60,sign*39,-8],[-62+dx,sign*(42+flare),height],[-64,sign*39,lower_anchor_z]])


def build(spec, *, length_mm=167, width_factor=1., yaw=0, axis=(0,0,1), k=0., shoe=None, sweep_deg=0.,
          slip=0.8, lower_anchor_z=-63):
    """Keep stock carrier and rail floor contacts. All added hinges are passive.

    A shoe tuple gives (fore–aft offset, lateral bulge, height, radius) in mm.
    The centre is a point on the outside of the right inherited rail. An equal
    shoe on the left gives the same envelope and mass on either side.
    """
    options={'width_factor':width_factor} if width_factor!=1. else {}
    SIZE['build'](spec,length_mm,1,1.,draped=True,**options)
    root=spec.body('study_link_0')
    if yaw:
        root.add_joint(name='recovery_pivot',type=mj.mjtJoint.mjJNT_HINGE,
            axis=np.asarray(axis)/np.linalg.norm(axis),range=np.deg2rad([-yaw,yaw]),limited=True,
            stiffness=k,damping=.00022,frictionloss=.00006,armature=.000002)
    if sweep_deg:
        a=np.deg2rad(sweep_deg)*.5
        root.quat=[np.cos(a),0,0,np.sin(a)]
    if shoe is None:return
    r=shoe[3]
    for sign in [-1,1]:
        surface=spec.body('tail_mount').add_body(name='recovery_skid_'+str(sign))
        # Beam runs from the headward rail shoulder to the footward shoulder.
        a,b,c=shoe_centers(shoe,sign,lower_anchor_z)
        for i,(p,q) in enumerate(((a,b),(b,c))):
            length=np.linalg.norm(q-p)
            # Capped tube wall 0.9 mm, density 1.20 g/cm3, bonded end allowance.
            kg=(np.pi*(r*r-(r-.9)**2)*length+.5*np.pi*r*r*1.0)*1.2e-6
            surface.add_geom(name=f'recovery_rail_{sign}_{i}',type=mj.mjtGeom.mjGEOM_CAPSULE,
                fromto=np.r_[p,q]*.001,size=[r*.001,0,0],mass=kg,
                friction=[slip,.005,.0001],contype=1,conaffinity=0,group=1)


def evaluate(runtime,variants,angle=90.,walk=None,speed=.45):
    checks=runpy.run_path(str(runtime/'tools/check-duck-hardware.py'))
    results=[]
    for name,params in variants.items():
        cfg=lambda spec:build(spec,**params)
        cls=POSTURE['simulation'](checks['WalkSimulation'],167,12,-48,-65,True,speed,
                                  build_attachment=cfg,transition_after_recovery=not walk)
        sims=[]
        class Trial(cls):
            def tick(self):
                if angle!=90 and self.data.time==0 and walk!='walk':
                    mj.mju_axisAngle2Quat(self.data.qpos[self.root+3:self.root+7],np.array([1.,0,0]),np.deg2rad(angle))
                return super().tick()
        def factory(*a,**kw):
            sim=Trial(*a,**kw);sims.append(sim);return sim
        out=checks['run_case'](walk or 'right',[],{},simulation_factory=factory)
        out['mean_trunk_forward_deg_last_5s']=round(float(np.mean([m[1] for m in sims[0].metrics if m[0]>=15])),2)
        passed=(checks['passes']('right',out,out) if not walk else out['upright'] and not out['fell'])
        row=dict(name=name,params=params,start_deg=angle,case=walk or 'right',speed_cmd=speed,passed=bool(passed),result=out)
        print(name,angle,out['recovered_s'],out['upright'],round(out['resumed_forward_m'] or 0,2),out['transitions'],flush=True)
        results.append(row)
    return results


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('runtime',type=Path);p.add_argument('input',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--speed',type=float,default=.45);p.add_argument('--angle',type=float,default=90);p.add_argument('--walk',action='store_true')
    a=p.parse_args()
    rows=evaluate(a.runtime,json.loads(a.input.read_text(encoding='utf-8-sig')),a.angle,'walk' if a.walk else None,a.speed)
    a.output.write_text(json.dumps(rows,indent=2,allow_nan=False)+'\n',encoding='utf-8')
