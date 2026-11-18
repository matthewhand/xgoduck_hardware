"""Compare the original and rounded guard on the concealed-head V5 with stock policies."""
import argparse
import hashlib
import json
from pathlib import Path
import runpy
import sys

import numpy as np

HERE=Path(__file__).resolve().parent
ACC=HERE.parents[1]
RAILS=HERE.parent/'removable-rails'
TAIL=ACC/'v5-raptor-tail'
RUNTIME=ACC.parents[1]/'xgoduck_runtime_arduino'
HEAD=ACC/'v5-face-plates/magnetic'
sys.path.insert(0,str(TAIL))
from rail_only import configure_rails
receiver=runpy.run_path(str(RAILS/'receiver_physics.py'))
checks=runpy.run_path(str(RUNTIME/'tools/check-duck-hardware.py'))
from walk_sim import load_sim_policy
roll=runpy.run_path(str(TAIL/'roll-after-removal.py'))['roll']
STOCK=json.loads((HEAD/'off-results.json').read_text())['trials']['stock']['policy']
head=json.loads((HEAD/'off-head-profile.json').read_text())


def configuration(profile):
    def apply(spec):
        receiver['configure_receiver'](spec)
        body=spec.body(head['parent'])
        for piece in head['parts'].values():
            s=-piece['sign']
            receiver['subtract'](body,s*piece['mass_kg'],piece['com_m'],s*np.array(piece['inertia_kg_m2']))
        configure_rails(spec,profile)
    return apply


def trial(config,case,angle=90):
    class Fall(checks['WalkSimulation']):
        def tick(self):
            if angle!=90 and self.data.time==0:
                self.mj.mju_axisAngle2Quat(self.data.qpos[self.root+3:self.root+7],
                    np.asarray(checks['FALLS'][case],dtype=float),np.deg2rad(angle))
            return super().tick()
    def create(vx,yaw,**kwargs):
        kwargs['configure']=config
        return Fall(vx,yaw,**kwargs)
    row=checks['run_case'](case,[],{},simulation_factory=create)
    row['pass']=bool(checks['passes'](case,row,STOCK[case]))
    return row


def run(profile,screen=False):
    apply=configuration(profile)
    cases={}
    for case in ('back','right','front','left','walk'):
        cases[case]=trial(apply,case)
        if screen and not cases[case]['pass']:return dict(cases=cases,rolls=[])
    rolls=[roll(checks['WalkSimulation'],load_sim_policy,apply,t) for t in (.2,.5,1.,2.)]
    return dict(cases=cases,rolls=rolls)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--screen',type=Path)
    parser.add_argument('--angles',action='store_true')
    parser.add_argument('--timing',action='store_true')
    args=parser.parse_args()
    if args.timing:
        path=HERE/'rounded-profile.json'
        apply=configuration(json.loads(path.read_text()))
        times=(0,.02,.10,.34,.76,1.50,2.50)
        rows={key:[roll(checks['WalkSimulation'],load_sim_policy,c,t) for t in times]
              for key,c in [('stock',None),('rounded',apply)]}
        (HERE/'launch-times.json').write_text(json.dumps(dict(profile_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),trials=rows),indent=2)+'\n')
    elif args.angles:
        apply=configuration(json.loads((HERE/'rounded-profile.json').read_text()))
        result={side:{angle:trial(apply,side,angle) for angle in (85,88,92,95)}
                for side in ('back','right','front','left')}
        (HERE/'release-angles.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    elif args.screen:
        for path in sorted(args.screen.glob('candidate-*.json')):
            result=run(json.loads(path.read_text()),True)
            print(path.stem,[k for k,v in result['cases'].items() if v['pass']],sum(v['pass_roll'] for v in result['rolls']),flush=True)
    else:
        result={name:run(json.loads(path.read_text())) for name,path in [('before',RAILS/'plain-guard-profile.json'),('after',HERE/'rounded-profile.json')]}
        result['policies_sha256']={k:hashlib.sha256((RUNTIME/'python'/f'xgoduck_{k}.onnx').read_bytes()).hexdigest() for k in ['walk','getup','roulade']}
        (HERE/'trials.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
