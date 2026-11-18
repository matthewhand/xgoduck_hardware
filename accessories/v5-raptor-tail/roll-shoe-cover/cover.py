"""Print the paired 167 mm trial runners integral with the removable V5 tail cover."""
import json
from pathlib import Path
import runpy
import numpy as np
from manifold3d import Manifold

HERE=Path(__file__).resolve().parent
tail=runpy.run_path(str(HERE.parent/'raptor-tail.py'))
rails=runpy.run_path(str(tail['BACKPACK']/'removable-rails/rail-cartridges.py'))


def capsule(a,b,r):
    return Manifold.batch_hull([tail['egg'](p,[r]*3) for p in [a,b]])


def build():
    _,fixed,cover,released,yoke=tail['base'](.25)
    def runner(sign):
        # Invert tail_physics.trunk() for the tested low-anchor shoe centres.
        a,b,c=(np.array([sign*x+.00628,z+39.25512,xf+57.33226]) for xf,x,z in
               [(-60,39,-8),(-62,67,-69),(-64,39,-66.5)])
        shell=capsule(a,b,4.5)+capsule(b,c,4.5)
        direction=lambda p,q:(q-p)/np.linalg.norm(q-p)
        shell-=capsule(a+direction(a,b)*1.2,b,3.6)+capsule(b,c+direction(c,b)*1.2,3.6)
        # Broaden the footward root into the removable cover edge.
        shell+=capsule(c,[sign*37.8,-26,-2.4],2.25)
        # Keep the separate M3 yoke seat, screws and driver entry accessible.
        shell-=tail['block'](-38.4,38.4,22.9,30.1,-14,-3.1)
        screw=Manifold.cylinder(21,3.6,3.6,48).translate((sign*34.5,26.5,-18))
        return shell-screw
    runners=runner(-1)+runner(1)
    return cover,released,yoke,fixed,runners


def main():
    cover,released,yoke,fixed,runners=build()
    _,host,rail,_,_,_=rails['build']()
    receiver=host+rail+rails['left_rail'](rail)
    swept=tail['CAD']['sweep'](released+runners,76)
    checks={}
    for name,rig in [('fixed rails',fixed),('removable rails',receiver)]:
        checks[name]=round(max(0,(swept^rig).volume()),5)
    checks['separate root yoke']=round(max(0,(runners^yoke).volume()),5)
    screws=list(tail['metal']().values())
    checks['M3 yoke hardware']=round(max(0,(runners^(screws[0]+screws[1])).volume()),5)
    if max(checks.values())>.0001:
        raise ValueError(checks)
    fitted=cover+runners
    tail['checked'](fitted)
    if len([p for p in fitted.decompose() if p.volume()>.001])!=1:
        raise ValueError('Disconnected runner')
    # Bed on the broad rear cover face; the curved runners will need local support.
    part=fitted.rotate((180,0,0))
    part=part.translate((0,0,-part.bounding_box()[2]))
    out=HERE/'tail-cover-with-runners.stl'
    tail['write_stl'](out,part)
    verify=runpy.run_path(str(HERE.parents[1]/'v5-dino-beak/verify-fit.py'))
    verify['check_export'](out)
    result={'service_travel_mm':76,'overlap_mm3':checks,
            'incremental_runner_mass_g_at_1p20':round((fitted-cover).volume()*.0012,3)}
    (HERE/'fit.json').write_text(json.dumps(result,indent=2)+'\n')
    print(result)

if __name__=='__main__':
    main()
