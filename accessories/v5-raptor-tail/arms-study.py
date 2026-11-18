"""Paired neck-collar, passive arm and draped-tail experiments. Simulation only."""
import argparse
import hashlib
import json
from pathlib import Path
import runpy

import mujoco as mj
import numpy as np

HERE=Path(__file__).resolve().parent
POSTURE=runpy.run_path(str(HERE/'posture-study.py'))
LENGTHS=(0,167,185,200)
LOADOUTS=('off','collar','arms')
DENSITY=1200


def capsule(body,name,start,end,radius,density):
    return body.add_geom(name=name,type=mj.mjtGeom.mjGEOM_CAPSULE,fromto=[*start,*end],
                         size=[radius,0,0],density=density,contype=5,conaffinity=0)


def build_arms(spec,loadout):
    if loadout=='off':
        return
    # Collar-local forward/left/up axes are -Y/-Z/+X of the lower neck link.
    # The base follows neck_pitch, not the trunk or the camera-carrying head.
    collar=spec.body('neck').add_body(name='study_collar',pos=[.020,0,.0145],
                                     quat=[.5,-.5,.5,-.5])
    # A rear opening lets an operator spring the necklace on and off. Give
    # each capsule its cylinder volume only; adjacent strips share their ends.
    angles=np.deg2rad(np.linspace(-148,148,19))
    points=np.column_stack([.016*np.cos(angles),.020*np.sin(angles),np.zeros_like(angles)])
    for i,(p,q) in enumerate(zip(points,points[1:])):
        radius=.0022
        length=np.linalg.norm(q-p)
        mass=DENSITY*np.pi*radius**2*length
        geom=capsule(collar,f'collar_strip_{i}',p,q,radius,DENSITY)
        geom.mass=mass
    for sign in (-1,1):
        side='left' if sign==1 else 'right'
        shoulder=[.008,sign*.018,0]
        collar.add_geom(name='collar_socket_'+side,type=mj.mjtGeom.mjGEOM_ELLIPSOID,
                             pos=shoulder,size=[.0035,.003,.0035],mass=.00020,contype=5,conaffinity=0)
        if loadout=='arms':
            arm=collar.add_body(name='study_arm_'+side,pos=shoulder)
            arm.add_joint(name='wobble_'+side,type=mj.mjtJoint.mjJNT_HINGE,
                          axis=[0,1,0],limited=True,range=np.deg2rad([-22,22]),
                          stiffness=.003,damping=.000045,frictionloss=.00001,armature=0)
            elbow=[.011,sign*.005,-.021]
            palm=[.028,sign*.004,-.014]
            capsule(arm,'arm_upper_'+side,[0,0,0],elbow,.0031,DENSITY)
            capsule(arm,'arm_fore_'+side,elbow,palm,.0026,DENSITY)
            arm.add_geom(name='arm_paw_'+side,type=mj.mjtGeom.mjGEOM_ELLIPSOID,
                         pos=palm,size=[.0045,.0035,.0027],density=DENSITY,contype=5,conaffinity=0)
    # Only one stock mesh per link participates in contact. Keep collar/arms
    # clear of the camera head and torso, and allow them to touch the floor.
    for meshname in ('trunk_base','head_roll'):
        choices=[g for g in spec.geoms if g.meshname==meshname and g.contype]
        if choices:
            (next((g for g in choices if g.group==3),choices[0])).conaffinity |= 4


def run_trial(checks,length,loadout,case,roll_shoes,arm_builder=build_arms):
    class WithArms(checks['WalkSimulation']):
        def __init__(self,vx,yaw,*,configure,**kwargs):
            def build(spec):
                configure(spec)
                arm_builder(spec,loadout)
            super().__init__(vx,yaw,configure=build,**kwargs)
    target,neck,head,speed=(12,-48,-65,.45) if case=='walk' or roll_shoes else (None,0,0,.2)
    attachment=None
    if length and roll_shoes:
        build=runpy.run_path(str(HERE/'recovery-shapes.py'))['build']
        attachment=lambda spec:build(spec,length_mm=length,width_factor=.90,shoe=[0,25,-69,4.5],lower_anchor_z=-66.5)
    base=POSTURE['simulation'](WithArms,length,target,neck,head,True,speed,
                              build_attachment=attachment,transition_after_recovery=roll_shoes and case=='right')
    captures=[]
    class Trial(base):
        def __init__(self,*args,**kwargs):
            super().__init__(*args,**kwargs)
            m=self.model
            groups={key:[] for key in ('collar','arms','tail')}
            for i in range(m.nbody):
                n=m.body(i).name or ''
                if n=='study_collar':
                    groups['collar'].append(i)
                elif n.startswith('study_arm_'):
                    groups['arms'].append(i)
                elif n.startswith(('study_link_','tail_','recovery_skid_')):
                    groups['tail'].append(i)
            self.groups=groups
            self.arm_q=[int(m.joint('wobble_'+s).qposadr[0]) for s in ('left','right')] if loadout=='arms' else []
            self.arm_geoms={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('arm_','collar_'))}
            self.sample=[]
            self.hinge=[]
            captures.append(self)
        def tick(self):
            value=super().tick()
            d,m=self.data,self.model
            if d.time>=5 and round(d.time/.02)%5==0:
                # Report mean quasi-static gravitational pitch moments in a
                # yaw-aligned frame about the midpoint of both sole sites.
                mj.mj_forward(m,d)
                front=d.xmat[self.trunk].reshape(3,3)[:2,0]
                front=front/max(np.linalg.norm(front),1e-10)
                feet=np.mean([d.site_xpos[m.site(s+'_foot').id,:2] for s in ('left','right')],axis=0)
                arm=(d.xipos[:,:2]-feet)@front
                mgx=m.body_mass*9.81*arm
                contact=any(self.floor in c.geom and self.arm_geoms.intersection(c.geom) for c in d.contact)
                self.sample.append([*[float(mgx[ids].sum()) for ids in self.groups.values()],
                                    1000*float((d.subtree_com[self.trunk,:2]-feet)@front), float(contact)])
                if self.arm_q:
                    self.hinge.append(np.rad2deg(d.qpos[self.arm_q]).copy())
            return value
    row=checks['run_case'](case,[],{},simulation_factory=Trial)
    s=captures[0]
    if case=='walk':
        measure=np.mean(s.sample,axis=0)
        row.update(collar_mNm=round(1000*measure[0],2),arms_mNm=round(1000*measure[1],2),
                   tail_mNm=round(1000*measure[2],2),system_com_ahead_of_feet_mm=round(measure[3],2),
                   collar_arm_floor_sample_pct=round(100*measure[4],2))
        if s.hinge:
            q=np.array(s.hinge)
            row['arm_angle_deg_minmax']=np.stack([q.min(0),q.max(0)],axis=1).round(2).tolist()
        post=np.array(s.metrics)
        row['trunk_forward_deg']=round(float(np.mean(post[post[:,0]>=5,1])),2)
    return row,{k:round(float(s.model.body_mass[v].sum())*1000,3) for k,v in s.groups.items()}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('runtime',type=Path)
    p.add_argument('output',type=Path)
    p.add_argument('--roll-shoes',action='store_true',help='Bilateral roll shoes, 90%% width, return to moderate lean after get-up')
    p.add_argument('--lengths',type=int,nargs='+',default=LENGTHS)
    args=p.parse_args()
    if 0 not in args.lengths:
        p.error('Include 0 to establish the stock gate reference')
    checks=runpy.run_path(str(args.runtime/'tools/check-duck-hardware.py'))
    runs=[]
    for length in sorted(set(args.lengths)):
        for loadout in LOADOUTS:
            row={'length_mm':length,'neck_loadout':loadout}
            for case in ('walk','right'):
                result,masses=run_trial(checks,length,loadout,case,args.roll_shoes)
                ref=result if not runs else runs[0][case]
                result['pass']=bool(checks['passes'](case,result,ref))
                row[case]=result
            row['added_mass_g']=masses
            runs.append(row)
            print(length,loadout,masses,round(row['walk']['forward_m'],3),row['right']['pass'],flush=True)
    dependencies={'arms-study.py':Path(__file__),'size-study.py':HERE/'size-study.py','posture-study.py':HERE/'posture-study.py',
                  'tail-physics.json':HERE/'tail-physics.json'}
    if args.roll_shoes:
        dependencies['recovery-shapes.py']=HERE/'recovery-shapes.py'
    for policy in ('xgoduck_walk.onnx','xgoduck_getup.onnx'):
        dependencies[policy]=args.runtime/'python'/policy
    digest={n:hashlib.sha256(f.read_bytes()).hexdigest() for n,f in dependencies.items()}
    args.output.write_text(json.dumps(dict(mount='compact bilateral roll shoes [0,25,-69,4.5], lower anchor -66.5 mm, 90% tail width' if args.roll_shoes else 'V1.1 fixed rail and tail cover',
        after_getup='ramp to 0.45 m/s, 12 deg trunk reference, neck/head -48/-65 deg' if args.roll_shoes else 'neutral 0.20 m/s walk',
        inputs_sha256=digest,conditions='20 s, flat, stock ONNX, nominal geometry; walk 0.45 m/s, 12 deg trunk reference, neck/head -48/-65 deg; right 90 deg start, neutral physical sensing and commands during get-up.',
        moment_reference='Yaw-aligned forward, around midpoint of both sole sites. Positive mNm is nose-down. Walk means cover seconds 5–20.',
        runs=runs),indent=2,allow_nan=False)+'\n',encoding='utf-8')


if __name__=='__main__':
    main()
