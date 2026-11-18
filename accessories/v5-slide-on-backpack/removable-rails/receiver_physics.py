"""Pack-off configuration with recessed V1.1 receiver and both rail cartridges removed."""
import json
from pathlib import Path
import numpy as np

PROFILE=Path(__file__).resolve().parent/'receiver-profile.json'


def subtract(body, remove, centroid, inertia):
    b=body
    old_mass=float(b.mass)
    old_center=np.array(b.ipos)
    tensor=np.asarray(b.fullinertia)
    if not np.all(np.isfinite(tensor)):
        raise ValueError('The stock trunk needs explicit body-frame fullinertia')
    i,j,k,xy,xz,yz=tensor
    old_tensor=np.array([[i,xy,xz],[xy,j,yz],[xz,yz,k]])
    center=np.asarray(centroid)
    new_mass=old_mass-remove
    new_center=(old_mass*old_center-remove*center)/new_mass
    def shift(v): return np.eye(3)*(v@v)-np.outer(v,v)
    new_tensor=(old_tensor+old_mass*shift(old_center-new_center)
                -np.asarray(inertia)-remove*shift(center-new_center))
    assert min(np.linalg.eigvalsh(new_tensor))>0
    b.mass=new_mass
    b.ipos=new_center
    b.fullinertia=new_tensor[[0,1,2,0,0,1],[0,1,2,1,2,2]]


def configure_receiver(spec):
    p=json.loads(PROFILE.read_text(encoding='utf-8'))
    subtract(spec.body('trunk_base'),p['mass_removed_kg'],p['removed_com_m'],p['removed_inertia_kg_m2'])


def exchange_installed_rails(spec):
    """After building the curved tail, replace only the original fixed rail mass and hulls."""
    import mujoco as mj
    path=Path(__file__).resolve().parents[2]/'v5-raptor-tail'
    ref=json.loads((path/'rail-only-physics.json').read_text())
    tail=json.loads((path/'tail-physics.json').read_text())['bodies']['mount']
    # The rail subset is derived first, then the geometry of the cover/yoke.
    for i,hull in enumerate(ref['collider_hulls_m']):
        if not np.array_equal(hull,tail['collider_hulls_m'][i]):
            raise ValueError('Original rail contact subset changed')
        spec.delete(spec.geom(f'tail_mount_{i}_collision'))
    mount=spec.body('tail_mount')
    subtract(mount,ref['mass_kg'],ref['com_m'],ref['inertia_kg_m2'])
    configure_receiver(spec)
    new=json.loads(PROFILE.read_text(encoding='utf-8'))['cartridges']
    body=mount.add_body(name='installed_rail_cartridges')
    body.explicitinertial=True
    body.mass=new['mass_kg'];body.ipos=new['com_m']
    I=np.asarray(new['inertia_kg_m2'])
    body.fullinertia=I[[0,1,2,0,0,1],[0,1,2,1,2,2]]
    for index,hull in enumerate(new['collider_hulls_m']):
        n='cartridge_'+str(index)
        spec.add_mesh(name=n,uservert=np.ravel(hull).tolist())
        body.add_geom(name=n+'_collision',type=mj.mjtGeom.mjGEOM_MESH,meshname=n,
                      mass=0,contype=1,conaffinity=0,group=3)
