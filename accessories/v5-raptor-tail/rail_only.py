"""V1.1 fixed rail addition only, with the backpack and its yoke removed."""
import argparse
import json
from pathlib import Path
import runpy
import numpy as np

HERE = Path(__file__).resolve().parent
PROFILE = HERE / 'rail-only-physics.json'


def generate():
    from tail_physics import trunk, triangles, mass_properties, collider_vertices
    cad = runpy.run_path(str(HERE/'raptor-tail.py'))
    stock, fixed, *_ = cad['base'](.25)
    rails = fixed-stock
    vertices = trunk(triangles(rails))
    density = 1240.
    mass, center, inertia = mass_properties(vertices,np.full(len(vertices),density))
    return dict(version=1,source='Fixed V1.1 top-entry rail enclosure minus stock V1.1 enclosure',
                parent='trunk_base',density_kg_m3=density,volume_cm3=rails.volume()/1000,
                mass_kg=mass,com_m=center.tolist(),inertia_kg_m2=inertia.tolist(),
                collider_hulls_m=collider_vertices(rails,np.zeros(3),'mount'))


def configure_rails(spec, profile=None):
    import mujoco as mj
    if profile is None: profile=json.loads(PROFILE.read_text(encoding='utf-8'))
    trunk=spec.body(profile['parent'])
    mount=trunk.add_body(name='unloaded_top_entry_rails')
    mount.explicitinertial=True
    mount.mass=profile['mass_kg']
    mount.ipos=profile['com_m']
    I=np.asarray(profile['inertia_kg_m2'])
    mount.fullinertia=I[[0,1,2,0,0,1],[0,1,2,1,2,2]]
    for idx,vertices in enumerate(profile['collider_hulls_m']):
        mesh='unloaded_rail_'+str(idx)
        spec.add_mesh(name=mesh,uservert=np.ravel(vertices).tolist())
        mount.add_geom(name=mesh+'_collision',type=mj.mjtGeom.mjGEOM_MESH,meshname=mesh,
                       mass=0,contype=1,conaffinity=0,group=3)


if __name__=='__main__':
    profile=generate()
    PROFILE.write_text(json.dumps(profile,indent=2)+'\n',encoding='utf-8')
    print('Added rail material',profile['mass_kg']*1000,'g',len(profile['collider_hulls_m']),'collision hulls')
