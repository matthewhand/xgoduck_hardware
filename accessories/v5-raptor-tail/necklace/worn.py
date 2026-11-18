"""Show the three-prong collar on the V5 simulation model and the same neck bare."""
from pathlib import Path
import runpy
import sys
from PIL import Image

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent))
runtime=HERE.parents[3]/'xgoduck_runtime_arduino'
sys.path.insert(0,str(runtime/'python'))
from walk_sim import WalkSimulation,load_recovery_scene

def main():
    draw=runpy.run_path(str(HERE.parent/'arms-render.py'))
    claw=runpy.run_path(str(HERE/'test-paws.py'))['claws']
    sim=draw['STUDY']['POSTURE']['simulation'](WalkSimulation,0,12,-48,-65,True,.45)(.45,0)
    for _ in range(300):
        sim.tick()
    sheet=Image.new('RGB',(832,340))
    for i,mode in enumerate(['bare','worn']):
        sheet.paste(draw['panel'](load_recovery_scene,mode,sim,claw),(i*416,0))
    sheet.save(HERE/'bare-and-worn.png',optimize=True)

if __name__=='__main__':main()
