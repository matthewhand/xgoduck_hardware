"""Replay right-side recovery with the printed runner mass and segmented ground contacts."""
from pathlib import Path
import json
import runpy
import sys
import numpy as np

HERE=Path(__file__).resolve().parent
TAIL=HERE.parent
RUNTIME=TAIL.parents[2]/'xgoduck_runtime_arduino'
PROFILE=HERE/'runner-profile.json'

def profile():
    from scipy.spatial import ConvexHull
    data=runpy.run_path(str(HERE/'cover.py'))
    cover,_,_,_,runners=data['build']()
    add=runners-cover
    props=runpy.run_path(str(TAIL/'tail_physics.py'))
    triangles=props['trunk'](props['triangles'](add))
    mass,com,I=props['mass_properties'](triangles,np.full(len(triangles),1200.))
    hulls=[]
    for sign in [-1,1]:
        for left in np.arange(30,74,4):
            section=add.trim_by_plane((sign,0,0),left).trim_by_plane((-sign,0,0),-left-4)
            for piece in section.decompose():
                if piece.volume()<.001:continue
                pts=props['trunk'](piece.to_mesh().vert_properties[:,:3])
                hulls.append(pts[ConvexHull(pts).vertices].round(8).tolist())
    return dict(mass_kg=mass,com_m=com.tolist(),inertia=I.tolist(),hulls_m=hulls)

def trial():
    import mujoco as mj
    source=json.loads(PROFILE.read_text())
    rails=TAIL.parent/'v5-slide-on-backpack'/'removable-rails'
    sys.path.insert(0,str(rails))
    from receiver_physics import exchange_installed_rails
    runner=runpy.run_path(str(TAIL/'recovery-shapes.py'))
    checks=runpy.run_path(str(RUNTIME/'tools/check-duck-hardware.py'))
    def cfg(spec):
        runner['build'](spec,length_mm=167,width_factor=.90)
        exchange_installed_rails(spec)
        b=spec.body('tail_mount').add_body(name='printed_runners')
        b.explicitinertial=True
        b.mass=source['mass_kg']
        b.ipos=source['com_m']
        b.fullinertia=np.asarray(source['inertia'])[[0,1,2,0,0,1],[0,1,2,1,2,2]]
        for i,h in enumerate(source['hulls_m']):
            name=f'printed_runner_{i}'
            spec.add_mesh(name=name,uservert=np.ravel(h))
            b.add_geom(name=name,type=mj.mjtGeom.mjGEOM_MESH,meshname=name,
                       contype=1,conaffinity=0,friction=[.8,.005,.0001],group=1)
    sim=runner['POSTURE']['simulation'](checks['WalkSimulation'],167,12,-48,-65,True,.45,
                                         build_attachment=cfg,transition_after_recovery=True)
    result=checks['run_case']('right',[],{},simulation_factory=sim)
    result['passed']=bool(checks['passes']('right',result,result))
    (HERE/'right-recovery.json').write_text(json.dumps(result,indent=2)+'\n')
    print(result)

if __name__=='__main__':
    if sys.argv[1:] == ['profile']:
        PROFILE.write_text(json.dumps(profile(),separators=(',',':'))+'\n')
    elif sys.argv[1:] == ['trial']:trial()
    else:raise SystemExit('Choose profile or trial')
