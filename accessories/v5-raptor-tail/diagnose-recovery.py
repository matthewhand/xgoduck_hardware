"""Isolate what changes the right-side recovery result on the stock ONNX."""
import argparse
import json
from pathlib import Path
import runpy

from tail_physics import HERE, configure, read_profile


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('runtime',type=Path)
    p.add_argument('short_profile',type=Path)
    p.add_argument('output',type=Path)
    args = p.parse_args()
    checks = runpy.run_path(str(args.runtime/'tools/check-duck-hardware.py'))
    sim_type = checks['WalkSimulation']
    stock = checks['run_case']('right',[],{})
    results = {}
    for label,profile in [('standard',read_profile()),('short',json.loads(args.short_profile.read_text()))]:
        for allowed in ['all','mount-only','none']:
            def setup(spec):
                configure(spec,profile,'passive')
                for geom in spec.geoms:
                    if geom.name.startswith('tail_') and (allowed=='none' or
                            (allowed=='mount-only' and not geom.name.startswith('tail_mount_'))):
                        geom.contype = geom.conaffinity = 0
            def create(vx,yaw,**kwargs):
                return sim_type(vx,yaw,**dict(kwargs,configure=setup))
            tr=checks['run_case']('right',[],{},simulation_factory=create)
            tr['pass'] = bool(checks['passes']('right',tr,stock))
            results[label+'-'+allowed]=tr
            print(label,allowed,tr['pass'],tr['recovered_s'],flush=True)
    args.output.write_text(json.dumps(dict(
        conditions='Same right-side start, stock walk and get-up. Toggle only contact participation of attachment geoms; all masses and inertias retained within either length.',
        trials=results),indent=2)+'\n',encoding='utf-8')


if __name__=='__main__':
    main()
