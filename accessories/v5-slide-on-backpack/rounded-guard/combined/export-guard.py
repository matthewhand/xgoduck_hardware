"""Export the tuned guard and run the original geometry checks into this comparison folder."""
import argparse
import json
from pathlib import Path
import runpy

HERE=Path(__file__).resolve().parent
KEYS=('shoulder_depth_mm','shoulder_center_y_mm','shoulder_half_width_mm',
      'side_flare_mm','side_center_y_mm','side_center_z_mm')


def export(params):
    base=runpy.run_path(str(HERE.parent/'rounded.py'))['export']
    return base(params, output=HERE, profile_name='guard-profile.json')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('parameters',nargs='*',type=float)
    args=p.parse_args()
    if args.parameters and len(args.parameters)!=6:p.error('Supply all six guard parameters')
    settings=args.parameters or [json.loads((HERE/'fit.json').read_text())[k] for k in KEYS]
    print(export(settings))
