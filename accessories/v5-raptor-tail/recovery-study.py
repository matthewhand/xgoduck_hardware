"""Right-side recovery with detachable roll shoes and a 167 mm passive tail."""
import argparse
import json
from pathlib import Path
import runpy

import mujoco as mj
import numpy as np

HERE=Path(__file__).resolve().parent
posture=runpy.run_path(str(HERE/'posture-study.py'))
shapes=runpy.run_path(str(HERE/'recovery-shapes.py'))
SHOE=(0,25,-69,4.5)
WIDTH=.90


def rig_class(base, revised=True, target=12, fallen=True, width=None, shoe=SHOE):
    width=(WIDTH if revised else 1.) if width is None else width
    def attachment(spec):
        shapes['build'](spec,length_mm=167,width_factor=width,shoe=shoe if revised else None,
                        lower_anchor_z=-66.5)
    return posture['simulation'](base,167,target,-48,-65,speed=.45,
                                 build_attachment=attachment,transition_after_recovery=fallen)


def start_on_side(rig,angle):
    root=rig.root
    mj.mju_axisAngle2Quat(rig.data.qpos[root+3:root+7],np.array([1.,0.,0.]),np.deg2rad(angle))
    mj.mj_forward(rig.model,rig.data)


def make_rig(base,revised=True,target=12):
    rig=rig_class(base,revised,target)(.2,0)
    start_on_side(rig,90)
    return rig


def trial(checks,revised=True,target=12,angle=90,width=None,shoe=SHOE):
    fallen=angle is not None
    kind=rig_class(checks['WalkSimulation'],revised,target,fallen,width,shoe)
    samples=[]
    class Trial(kind):
        def tick(self):
            if fallen and not self.data.time:
                start_on_side(self,angle)
            return super().tick()
    def create(*args,**kwargs):
        sim=Trial(*args,**kwargs)
        samples.append(sim)
        return sim
    case='right' if fallen else 'walk'
    r=checks['run_case'](case,[],{},simulation_factory=create)
    m=np.array(samples[0].metrics)
    trunk,neck,bill=m[m[:,0]>=15,1:4].mean(axis=0)
    r.update(mean_trunk_deg=round(trunk,2),mean_neck_deg=round(neck,2),mean_bill_deg=round(bill,2))
    # Absolute forward distance along resumption heading avoids crediting the
    # displacement of the initial fall. The standing screen uses a fixed gait gate.
    r['pass']=bool(checks['passes']('right',r,r) if fallen else
                   r['upright'] and not r['fell'] and r['forward_m']>=1.5 and r['both_feet']>=10)
    return dict(revised=revised,reference_deg=target,start_deg=angle,result=r)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runtime',type=Path)
    parser.add_argument('output',type=Path)
    args=parser.parse_args()
    checks=runpy.run_path(str(args.runtime/'tools/check-duck-hardware.py'))
    report={'length_mm':167,'shoe':SHOE,'width_factor':WIDTH,'command_m_s':.45}
    report['recovery']=[trial(checks,revised,angle=a) for revised in [False,True]
                        for a in (80,85,90,95,100)]
    report['geometry_controls']=[trial(checks,False,width=WIDTH),trial(checks,True,width=1)]
    report['lean']=[trial(checks,target=t,angle=None) for t in range(8,31)]
    report['lean_after_fall']=[trial(checks,target=t) for t in range(10,27)]
    args.output.write_text(json.dumps(report,allow_nan=False,indent=2)+'\n',encoding='utf8')


if __name__=='__main__':
    main()
