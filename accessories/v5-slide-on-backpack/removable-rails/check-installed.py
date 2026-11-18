"""Compare right-side recovery with original and removable rail hardware installed."""
import argparse,json,runpy,sys
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent
TAIL=HERE.parents[1]/'v5-raptor-tail'
sys.path.insert(0,str(HERE))
from receiver_physics import exchange_installed_rails


def main():
    p=argparse.ArgumentParser();p.add_argument('runtime',type=Path);args=p.parse_args()
    runner=runpy.run_path(str(TAIL/'recovery-shapes.py'))
    checks=runpy.run_path(str(args.runtime/'tools/check-duck-hardware.py'))
    report={}
    for variant in ['fixed','cartridges']:
        def cfg(spec):
            runner['build'](spec,length_mm=167,width_factor=1,shoe=[0,25,-69,4.5],lower_anchor_z=-66.5)
            if variant=='cartridges':exchange_installed_rails(spec)
        cls=runner['POSTURE']['simulation'](checks['WalkSimulation'],167,12,-48,-65,True,.45,
                                            build_attachment=cfg,transition_after_recovery=True)
        sims=[]
        def make(*a,**kw):
            sim=cls(*a,**kw);sims.append(sim);return sim
        out=checks['run_case']('right',[],{},simulation_factory=make)
        out['passed']=bool(checks['passes']('right',out,out))
        out['mean_trunk_forward_deg_last_5s']=float(np.mean([m[1] for m in sims[0].metrics if m[0]>=15]))
        report[variant]=out
        print(variant,out,flush=True)
    (HERE/'installed-right-recovery.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')

if __name__=='__main__':main()
