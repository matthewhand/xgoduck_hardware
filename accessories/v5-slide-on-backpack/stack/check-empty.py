"""Test zero, one and two empty cartridges with the 167 mm roll-shoe tail."""
import argparse
import json
from pathlib import Path
import runpy

import mujoco as mj
import numpy as np

HERE=Path(__file__).resolve().parent
checks=runpy.run_path(str(HERE.parent/'check-shoulder-pointer.py'))
cad=checks['STACK']


def fit(spec,count):
    checks['add_stack'](spec,'removed',True)
    for level in range(count):
        offset=[0,level*cad['STEP'],0]
        solids=[s.translate(offset) for s in (cad['cartridge'](),cad['pin'](38 if level==0 else 30))]
        surfaces=np.concatenate([checks['trunk'](checks['triangles'](s)) for s in solids])
        mass,com,inertia=checks['mass_properties'](surfaces,np.full(len(surfaces),1240))
        body=spec.body('trunk_base').add_body(name=f'empty_level_{level+1}')
        body.explicitinertial=True
        body.mass=mass
        body.ipos=com
        body.fullinertia=inertia[[0,1,2,0,0,1],[0,1,2,1,2,2]]
        for n,solid in enumerate(solids):
            for i,vertices in enumerate(checks['collider_vertices'](solid,np.zeros(3),'mount')):
                name=f'empty_{level}_{n}_{i}'
                spec.add_mesh(name=name,uservert=np.ravel(vertices))
                body.add_geom(name=name,type=mj.mjtGeom.mjGEOM_MESH,meshname=name,contype=1,conaffinity=0,group=1)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runtime',type=Path)
    parser.add_argument('output',type=Path)
    args=parser.parse_args()
    test=runpy.run_path(str(args.runtime/'tools/check-duck-hardware.py'))
    baseline={case:test['run_case'](case,[],{}) for case in ('walk','right')}
    rows=[]
    for count in range(3):
        def factory(vx,yaw,**kwargs):
            kwargs['configure']=lambda spec:fit(spec,count)
            return test['WalkSimulation'](vx,yaw,**kwargs)
        for case in ('walk','right'):
            trial=test['run_case'](case,[],{},simulation_factory=factory)
            passed=bool(test['passes'](case,trial,baseline[case]))
            row={'empty_cartridges':count,'case':case,'passed':passed,'outcome':trial}
            rows.append(row)
            print(count,case,passed,round(trial['forward_m'],3),trial['recovered_s'],flush=True)
    args.output.write_text(json.dumps({'stock':baseline,'tail_mm':167,'floor':'level','policy':'stock walk and get-up ONNX',
        'walk_command_m_s':.2,'duration_s':20,'runs':rows},indent=2,allow_nan=False)+'\n',encoding='utf-8')


if __name__=='__main__':
    main()
