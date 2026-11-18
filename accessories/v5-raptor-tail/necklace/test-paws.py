"""A/B the stock policies with three-finger paws on the passive-necklace model."""
import json
from pathlib import Path
import runpy
import numpy as np
import sys

HERE=Path(__file__).resolve().parent
TAIL=HERE.parent
sys.path.insert(0,str(TAIL))
s=runpy.run_path(str(TAIL/'arms-study.py'))
digits=runpy.run_path(str(HERE/'digits.py'))['digits']

def claws(spec, loadout):
    s['build_arms'](spec,loadout)
    if loadout!='arms':return
    for sign,side in [(-1,'right'),(1,'left')]:
        body=spec.body('study_arm_'+side)
        for k,digit in enumerate(digits(sign)):
            for j,((p,r),(q,t)) in enumerate(zip(digit[:-1],digit[1:])):
                a,b=np.asarray(p)*.001,np.asarray(q)*.001
                radius=(r+t)*.0005
                geom=s['capsule'](body,f'arm_digit_{side}_{k}_{j}',a,b,radius,1200.)
                geom.mass=1200*np.pi*radius**2*np.linalg.norm(a-b)

def main():
    runtime=TAIL.parents[2]/'xgoduck_runtime_arduino'
    checks=runpy.run_path(str(runtime/'tools/check-duck-hardware.py'))
    base=s['run_trial']
    results={}
    for name,builder in [('smooth',s['build_arms']),('three_claws',claws)]:
        results[name]={}
        for case in ('walk','right'):
            result,mass=base(checks,167,'arms',case,True,builder)
            results[name][case]=result
            results[name]['model_mass_g']=mass
    (HERE/'policy-trials.json').write_text(json.dumps(results,indent=2)+'\n')
    for name,p in results.items():print(name,p['walk']['forward_m'],p['right']['recovered_s'],p['right']['resumed_forward_m'])

if __name__=='__main__':main()
