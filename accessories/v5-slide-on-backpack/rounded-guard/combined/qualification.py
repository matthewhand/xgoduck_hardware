"""Compare two guards with the same concealed-mount, integral-tooth head and stock policies."""
import argparse
import hashlib
import json
from pathlib import Path
import runpy


HERE=Path(__file__).resolve().parent
RG=HERE.parent
G=runpy.run_path(str(RG/'check-guard.py'))
TIMES=(0,.02,.10,.20,.34,.50,.76,1.,1.50,2.,2.50)
ANGLES=(85,88,90,92,95)


def config(guard_profile,head_profile):
    configure_head=runpy.run_path(str(HERE/'head.py'))['configure']
    def apply(spec):
        G['receiver']['configure_receiver'](spec)
        configure_head(spec,head_profile)
        G['configure_rails'](spec,guard_profile)
    return apply


def cases():
    return (['walk']+[f'{direction}{a}' for direction in ('back','right','front','left') for a in ANGLES]
            +[f'roll{round(100*t):03d}' for t in TIMES])


def case(apply,key):
    if key.startswith('roll'):
        return G['roll'](G['checks']['WalkSimulation'],G['load_sim_policy'],apply,int(key[4:])/100)
    if key=='walk':
        return G['trial'](apply,'walk')
    return G['trial'](apply,key[:-2],int(key[-2:]))


def passed(row):
    return bool(row['pass_roll'] if 'pass_roll' in row else row['pass'])


def evaluate(guard,head):
    apply=config(guard,head)
    return {k:case(apply,k) for k in cases()}


def read(p):
    return json.loads(p.read_text())


def sha(p):
    data=p.read_bytes()
    if p.suffix == '.json':
        data=data.replace(b'\r\n', b'\n')
    return hashlib.sha256(data).hexdigest()


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--before-only',action='store_true')
    args=p.parse_args()
    head=read(HERE/'head-profile.json')
    profiles={'before':RG/'rounded-profile.json','after':HERE/'guard-profile.json'}
    if args.before_only:
        profiles.pop('after')
    result={}
    for key,path in profiles.items():
        rows=evaluate(read(path),head)
        result[key]={'teeth':rows}
        print(key,'passed',sum(map(passed,rows.values())),'of',len(rows),'failed',[k for k,v in rows.items() if not passed(v)],flush=True)
    inputs=[HERE/'head-profile.json',RG/'rounded-profile.json',G['RAILS']/'receiver-profile.json',
            *[G['RUNTIME']/'python'/f'xgoduck_{n}.onnx' for n in ['walk','getup','roulade']]]
    if not args.before_only:
        inputs.append(HERE/'guard-profile.json')
    result['inputs_sha256']={p.name:sha(p) for p in inputs}
    dest='baseline-only.json' if args.before_only else 'qualification-results.json'
    (HERE/dest).write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
