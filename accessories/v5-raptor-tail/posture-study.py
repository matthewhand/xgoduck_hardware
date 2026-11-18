"""Screen a down-swept backpack tail with a forward neck and counter-pitched head."""
import argparse
import json
import runpy
from pathlib import Path

import mujoco as mj
import numpy as np

HERE=Path(__file__).resolve().parent
sizing=runpy.run_path(str(HERE/'size-study.py'))


def simulation(base, length, target, neck, head, draped=True, speed=.2, *,
               build_attachment=None, transition_after_recovery=False):
    class PostureTrial(base):
        def __init__(self,vx,yaw,**kwargs):
            kwargs['configure']=build_attachment or (lambda spec:sizing['build'](spec,length,1,1.,draped=draped))
            super().__init__(speed,yaw,**kwargs)
            m=self.model
            if target is not None:
                # Express measured gravity relative to the desired pitch for
                # the walking network. Physics and get-up sensing stay physical.
                interface=self.walk[0]
                session=interface.session
                rig=self
                class TiltReference:
                    def run(self,outputs,feed):
                        key=next(iter(feed))
                        obs=feed[key].copy()
                        angle=np.deg2rad(target)*rig.posture_ramp()
                        x,z=obs[0,[3,5]]
                        obs[0,3]=np.cos(angle)*x+np.sin(angle)*z
                        obs[0,5]=-np.sin(angle)*x+np.cos(angle)*z
                        return session.run(outputs,{key:obs})
                interface.session=TiltReference()
            self.jaw=m.body('jaw_soft').id
            self.neck_base=m.body('neck').id
            self.neck_end=m.body('neck_pitch').id
            self.geoms={g for g in range(m.ngeom) if (m.geom(g).name or '').startswith('candidate_')}
            self.metrics=[]
            self.contacts=0
            self.substeps=0

        def posture_ramp(self):
            if self.recovery.phase!='walk':
                return 0.
            if transition_after_recovery:
                if self.resumed_at_s is None:
                    return 0.
                return np.clip((self.data.time-self.resumed_at_s-.3)/1.5,0,1)
            return np.clip((self.data.time-2)/1.5,0,1)

        def tick(self):
            m,d=self.model,self.data
            ramp=self.posture_ramp()
            if transition_after_recovery:
                self.command=(.2+(speed-.2)*ramp,0,0)
            self.obs[0,51:53]=ramp*np.deg2rad([neck,head])
            def contacts(sim,feet):
                if d.time>=5:
                    self.substeps+=1
                    self.contacts+=any(self.floor in c.geom and bool(self.geoms.intersection(c.geom)) for c in d.contact)
            result=super().tick(observe_contacts=contacts)
            if round(d.time/.02)%5==0:
                mj.mj_forward(m,d)
                rot=d.xmat[self.trunk].reshape(3,3)
                heading=rot[:2,0]/max(np.linalg.norm(rot[:2,0]),1e-8)
                neck_vec=d.xpos[self.neck_end]-d.xpos[self.neck_base]
                bill=-d.xmat[self.jaw].reshape(3,3)[:,2]
                row=[d.time,np.rad2deg(np.arcsin(np.clip(-rot[2,0],-1,1))),
                     np.rad2deg(np.arctan2(neck_vec[:2]@heading,neck_vec[2])),
                     np.rad2deg(np.arcsin(np.clip(bill[2],-1,1)))]
                if length and draped:
                    tip=d.site_xpos[m.site('tail_tip').id]
                    root=d.xpos[m.body('study_link_0').id]
                    row.extend([(root[:2]-tip[:2])@heading*1000,tip[2]*1000])
                self.metrics.append(row)
            return result
    return PostureTrial


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runtime',type=Path)
    parser.add_argument('output',type=Path)
    args=parser.parse_args()
    checks=runpy.run_path(str(args.runtime/'tools/check-duck-hardware.py'))
    report=[]
    def run(length,target,neck,head,case,draped=True,speed=.2):
        sims=[]
        kind=simulation(checks['WalkSimulation'],length,target,neck,head,draped,speed)
        def create(*args,**kwargs):
            sims.append(kind(*args,**kwargs))
            return sims[-1]
        outcome=checks['run_case'](case,[],{},simulation_factory=create)
        sim=sims[0]
        sample=np.array(sim.metrics)
        settled=sample[sample[:,0]>=5]
        fields=['trunk_forward_deg','neck_forward_deg','bill_above_horizontal_deg','tip_aft_mm','tip_height_mm']
        posture=dict(zip(fields,np.round(settled[:,1:].mean(axis=0),2)))
        if length and draped:
            before=sample[(sample[:,0]>=.5)&(sample[:,0]<2)]
            posture.update(before_tip_aft_mm=round(float(before[:,4].mean()),2),
                           min_tip_height_mm=round(float(settled[:,5].min()),2))
        row=dict(length_mm=length,draped=draped,speed_cmd_m_s=speed,trunk_target_deg=target,neck_cmd_deg=neck,
                 head_cmd_deg=head,case=case,outcome=outcome,**posture,
                 tail_ground_percent=round(100*sim.contacts/max(1,sim.substeps),2))
        report.append(row)
        print(length,target,case,round(outcome['forward_m'],2),posture,flush=True)
    run(132,None,-25,0,'walk',False)
    for length in (0,158,167,176):
        for target in (None,8,12,16,22):
            run(length,target,-48,-65,'walk')
        if length:
            run(length,None,0,0,'right')
    for speed in (.35,.45):
        for length in (0,167):
            for target in (12,16):
                run(length,target,-48,-65,'walk',speed=speed)
    for row in report:
        if row['case']=='walk' and row['draped']:
            ref=next(r for r in report if r['length_mm']==0 and r['trunk_target_deg']==row['trunk_target_deg']
                     and r['speed_cmd_m_s']==row['speed_cmd_m_s'])
            row['walk_pass']=bool(checks['passes']('walk',row['outcome'],ref['outcome']))
    args.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf8')


if __name__=='__main__':
    main()
