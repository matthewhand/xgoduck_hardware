"""Compare a plain protective cover and the flush V1.1 receiver with the stock rear."""
import json
from pathlib import Path
import runpy
import sys

import numpy as np

HERE=Path(__file__).resolve().parent
TAIL=HERE.parents[1]/'v5-raptor-tail'
sys.path.insert(0,str(TAIL))
sys.path.insert(0,str(HERE.parents[1]))
RUNTIME=TAIL.parents[2]/'xgoduck_runtime_arduino'
PROFILE=HERE/'plain-guard-profile.json'


def build():
    rails=runpy.run_path(str(HERE/'rail-cartridges.py'))
    stock,receiver,rail,*_=rails['build']()
    fixed,cover,rigid,tooth=rails['C']['build'](stock,.25,charging_contacts=False)
    track=rail+rails['left_rail'](rail)
    slide=rails['C']['sweep'](rigid+tooth.translate((0,0,-.85)),76)
    for fixture in (receiver+track,fixed):
        assert (slide^fixture).volume()<1e-4
    temp=cover.rotate((180,0,0))
    path=HERE/'print/plain-protective-cover.stl'
    rails['C']['write_stl'](path,temp.translate((0,0,-temp.bounding_box()[2])))
    verify=runpy.run_path(str(TAIL.parent/'v5-dino-beak/verify-fit.py'))
    verify['check_export'](path)
    from tail_physics import trunk,triangles,mass_properties,collider_vertices
    vertices=trunk(triangles(cover+track))
    mass,com,I=mass_properties(vertices,np.full(len(vertices),1240.))
    hulls=collider_vertices(cover,np.zeros(3),'mount')+collider_vertices(track,np.zeros(3),'mount')
    PROFILE.write_text(json.dumps(dict(parent='trunk_base',mass_kg=mass,com_m=com.tolist(),
                         inertia_kg_m2=I.tolist(),collider_hulls_m=hulls),separators=(',',':'))+'\n')
    from cad_figure import render,label,bg
    from PIL import Image
    fig=Image.new('RGB',(1080,340),bg)
    runner=runpy.run_path(str(TAIL/'roll-shoe-cover/cover.py'))
    tail_cover,_,_,_,shoes=runner['build']()
    for i,(components,title) in enumerate([
        ([(receiver,[195,193,173])],'1   Recessed receiver'),
        ([(receiver,[195,193,173]),(track,[104,112,114]),(cover,[95,146,137])],'2   Everyday guard'),
        ([(tail_cover,[95,146,137]),(shoes,[213,153,70])],'3   Lift-off runner cover')]):
        im,_=render([(triangles(s),col) for s,col in components],360,252,[.55,.39,-1],[0,1,0])
        fig.paste(im,(i*360,63))
        label(fig,(i*360+12,21),title,19,True)
    fig.save(HERE/'rear-configurations.png',optimize=True)


def test():
    from rail_only import configure_rails
    cfg=runpy.run_path(str(HERE/'receiver_physics.py'))['configure_receiver']
    roll=runpy.run_path(str(TAIL/'roll-after-removal.py'))['roll']
    checks=runpy.run_path(str(RUNTIME/'tools/check-duck-hardware.py'))
    from walk_sim import load_sim_policy
    profile=json.loads(PROFILE.read_text())
    def guarded(spec):
        cfg(spec)
        configure_rails(spec,profile)
    result={}
    for name,config in [('receiver',cfg),('guarded',guarded)]:
        result[name]=[roll(checks['WalkSimulation'],load_sim_policy,config,t) for t in (.2,.5,1.,2.)]
        print(name,[(row['turn_deg'],row['pass_roll']) for row in result[name]])
    (HERE/'plain-guard-roll.json').write_text(json.dumps(result,indent=2)+'\n')


if __name__=='__main__':
    if sys.argv[1:]==['build']:build()
    elif sys.argv[1:]==['test']:test()
    else:raise SystemExit('Choose build or test')
