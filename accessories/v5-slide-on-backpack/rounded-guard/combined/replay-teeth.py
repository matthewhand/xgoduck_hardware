"""Replay matched 95° right-side get-ups and 0.34 s roll launches with co-printed teeth."""
from contextlib import ExitStack
import hashlib
import json
from pathlib import Path
import runpy

import mujoco as mj
import numpy as np

HERE=Path(__file__).resolve().parent
REPLAY=runpy.run_path(str(HERE.parent/'replay.py'))
CAD=REPLAY['CAD']
BEAK=HERE.parents[2]/'v5-dino-beak'
PARAMETERS=('shoulder_depth_mm','shoulder_center_y_mm','shoulder_half_width_mm',
            'side_flare_mm','side_center_y_mm','side_center_z_mm')


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def tooth_visuals():
    printer=runpy.run_path(str(BEAK/'coprint/coprint.py'))
    registration=runpy.run_path(str(BEAK/'export-preview.py'))['registered']
    meshes=[]
    for side in ('upper','lower'):
        teeth=printer['build'](side)['teeth']
        assert len(teeth)==8
        for i,tooth in enumerate(teeth):
            mesh=tooth.simplify(.012).to_mesh()
            vertices=registration(mesh.vert_properties[:,:3],side)*.001
            meshes.append((f'coprint_{side}_{i}',vertices,mesh.tri_verts))

    def add(spec):
        body=spec.body('jaw_soft')
        for name,vertices,faces in meshes:
            spec.add_mesh(name=name,uservert=vertices.ravel(),userface=faces.ravel())
            body.add_geom(name=name,type=mj.mjtGeom.mjGEOM_MESH,meshname=name,rgba=[1,.61,.18,1],
                          mass=0,contype=0,conaffinity=0,group=1)
    return add


def generate(config,head_profile,guard_profiles,parameters,results,target):
    before,after=results['before']['teeth'],results['after']['teeth']
    comparison={label:{'cases':{'right':group['right95']},'rolls':[group['roll034']]}
                for label,group in [('before',before),('after',after)]}
    phases={}
    def capture(profile,case,settle):
        row,slots,names=REPLAY['record'](profile,case,settle,configure=config(profile,head_profile),release_angle=95)
        side='before' if profile is guard_profiles[0] else 'after'
        phase_log=[{'s':round(i*.02,2),'policy_phase':slots[i][1]} for i in sorted(slots)
                   if i==0 or slots[i][1]!=slots[i-1][1]]
        if settle is not None:
            launch=round(settle/.02)
            assert slots[launch][1]=='roulade' and slots[launch+85][1]!='roulade'
        phases.setdefault(case,{})[side]=phase_log
        return row,slots,names
    _,receiver,rails,*_=CAD['base']()
    add_teeth=tooth_visuals()
    with ExitStack() as stack:
        panels=[REPLAY['Panel'](REPLAY['model'](receiver,rails,CAD['rounded'](*values)[0],add_visual=add_teeth),stack)
                for values in parameters]
        manifest=REPLAY['film'](panels,guard_profiles,[('right',None),('roll',.34)],comparison,target,
            reader=capture,labels=('Current · teeth','Revised · teeth'),release_angle=95)
    for event in manifest['events']:
        event['phase_transitions']=phases[event['case']]
        if event['case']=='right':
            event['release_angle_deg']=95
        else:
            event['roll_launch_slot']=17
    return manifest


def main():
    q=runpy.run_path(str(HERE/'qualification.py'))
    inputs=[HERE.parent/'rounded-profile.json',HERE/'guard-profile.json']
    fits=[read(HERE.parent/'fit.json'),read(HERE/'fit.json')]
    params=[[f[n] for n in PARAMETERS] for f in fits]
    report=HERE/'qualification-results.json'
    head=HERE/'head-profile.json'
    published=read(report)
    policy_files=[REPLAY['TEST']['RUNTIME']/'python'/f'xgoduck_{n}.onnx' for n in ('walk','getup','roulade')]
    for source in [*inputs,head,REPLAY['TEST']['RAILS']/'receiver-profile.json',*policy_files]:
        if q['sha'](source)!=published['inputs_sha256'][source.name]:
            raise ValueError(f'Published trials used a different {source.name}')
    target=HERE/'media/teeth-guard.webp'
    target.parent.mkdir(exist_ok=True)
    metadata=generate(q['config'],read(head),[read(f) for f in inputs],params,published,target)
    metadata['inputs_sha256']={f'{f.parent.name}/{f.name}':q['sha'](f)
                               for f in [*inputs,head,report,HERE.parent/'fit.json',HERE/'fit.json']}
    metadata['sha256']=hashlib.sha256(target.read_bytes()).hexdigest()
    (target.parent/'teeth-guard-replay.json').write_text(json.dumps(metadata,indent=2)+'\n',encoding='utf-8')
    print(target.name,metadata['bytes'],metadata['sha256'])


if __name__=='__main__':
    main()
