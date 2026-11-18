"""Generate a neutral-jaw face accessory profile, then replay stock walk/get-up."""
import argparse
import hashlib
import json
from pathlib import Path
import runpy

import numpy as np


HERE = Path(__file__).resolve().parent
RUNTIME = HERE.parents[2] / 'xgoduck_runtime_arduino'
PROFILE = HERE / 'motion-profile.json'


def generate(runtime):
    from scipy.spatial import ConvexHull
    from manifold3d import Manifold
    cad = runpy.run_path(str(HERE / 'face-plates.py'))
    preview = runpy.run_path(str(cad['BEAK'] / 'export-preview.py'))
    # Checks the hash and connected components of the head used for registration.
    list(preview['stock_head'](runtime))
    props = runpy.run_path(str(HERE.parent / 'v5-raptor-tail' / 'tail_physics.py'))
    face, jaw, mask, beard, _, _ = cad['build']()
    delta_face = face - cad['cad']['read_stock']('FACE')
    delta_jaw = jaw - cad['cad']['read_stock']('JAW')
    face_to_head = ((0,-1,0,0), (0,0,1,cad['FACE_Y']), (-1,0,0,33.258))
    pieces = [
        ('brow_boss', 'upper', delta_face.transform(face_to_head), 1200),
        ('chin_boss', 'lower', delta_jaw, 1200),
        ('mask', 'upper', cad['front'](mask), 1200),
        ('beard', 'lower', cad['chin'](beard), 1200)]
    for side, pts, transform, z in [
            ('upper', cad['FACE_POINTS'], cad['front'], cad['BROW_EXTENSION']),
            ('lower', cad['CHIN_POINTS'], cad['chin'], 0)]:
        for i, (x,y) in enumerate(pts):
            nut = (Manifold.cube((4,4,1.6)).translate((x-2,y-2,1.40))
                   - cad['cyl'](x,y,1,3,1.05))
            pair = (cad['metal']([(x,y)]) + nut).translate((0,0,z))
            pieces.append((f'{side}_fastener_{i+1}',side,transform(pair),7850))

    meshes, densities, hulls, masses = [], [], {}, {}
    for name, side, solid, density in pieces:
        register = lambda pts: preview['registered'](pts, side) / 1000
        tris = register(props['triangles'](solid))
        meshes.append(tris)
        densities.append(np.full(len(tris), density))
        masses[name] = solid.volume() * 1e-6 * density
        if 'fastener' in name:
            continue  # Six recessed pairs add mass but do not protrude from the plates.
        # Preserve the brow and chin silhouette with small convex cross-sections.
        for component in solid.decompose():
            lo, _, _, hi, _, _ = component.bounding_box()
            if component.volume() < .01:
                continue
            sections = np.linspace(lo-.01,hi+.01,1+int(np.ceil((hi-lo)/8)))
            for l, r in zip(sections,sections[1:]):
                piece = component.trim_by_plane((1,0,0),l).trim_by_plane((-1,0,0),-r)
                if piece.volume() < .01:
                    continue
                p = register(piece.to_mesh().vert_properties[:,:3])
                pts = p[ConvexHull(p).vertices]
                hulls.setdefault(name, []).append(pts.round(8).tolist())
    mass, com, inertia = props['mass_properties'](np.concatenate(meshes), np.concatenate(densities))
    assert abs(mass*1000 - sum(masses.values())) < .001
    return dict(mass_kg=mass, com_m=com.tolist(), inertia_kg_m2=inertia.tolist(),
                materials_kg_m3={'printed':1200, 'steel':7850}, masses_g=masses, hulls_m=hulls,
                head_sha256=preview['HEAD_SHA'])


def configure(spec, profile):
    import mujoco as mj
    body = spec.body('jaw_soft').add_body(name='mask_and_beard')
    body.explicitinertial = True
    body.mass = profile['mass_kg']
    body.ipos = profile['com_m']
    I = np.asarray(profile['inertia_kg_m2'])
    body.fullinertia = I[[0,1,2,0,0,1],[0,1,2,1,2,2]]
    for group, hulls in profile['hulls_m'].items():
        for i, hull in enumerate(hulls):
            name = f'faceplate_{group}_{i}'
            spec.add_mesh(name=name, uservert=np.ravel(hull).tolist())
            body.add_geom(name=name, type=mj.mjtGeom.mjGEOM_MESH, meshname=name,
                          mass=0, contype=3, conaffinity=0, group=3)


def trials(runtime, profile):
    checks = runpy.run_path(str(runtime / 'tools' / 'check-duck-hardware.py'))
    results = {}
    for variant in ('stock', 'mask_and_beard'):
        rows = results[variant] = {}
        for case in ('walk','back','front','left','right'):
            touches = {}
            class Simulation(checks['WalkSimulation']):
                def __init__(self, vx, yaw, **kwargs):
                    if variant == 'mask_and_beard':
                        kwargs['configure'] = lambda spec: configure(spec, profile)
                    super().__init__(vx, yaw, **kwargs)
                    self.labels = {i: self.model.geom(i).name[10:].rsplit('_',1)[0]
                                   for i in range(self.model.ngeom)
                                   if (self.model.geom(i).name or '').startswith('faceplate_')}
                def sample(self, sim, feet):
                    names = set()
                    for c in sim.data.contact:
                        a,b = c.geom
                        if a == sim.floor and b in self.labels:
                            names.add(self.labels[b])
                        if b == sim.floor and a in self.labels:
                            names.add(self.labels[a])
                    for name in names:
                        touches[name] = touches.get(name,0) + 1
                def tick(self):
                    return super().tick(observe_contacts=self.sample if self.labels else None)

            row = checks['run_case'](case, [], {}, simulation_factory=Simulation)
            row['passed'] = bool(checks['passes'](case,row,row if variant=='stock' else results['stock'][case]))
            row['accessory_floor_contact_s'] = {k:round(v*.005,3) for k,v in touches.items()}
            rows[case] = row
            print(variant, case, row['passed'], row['recovered_s'], row['forward_m'], flush=True)
    assert abs(results['mask_and_beard']['walk']['mass_g'] - results['stock']['walk']['mass_g']
               - 1000 * profile['mass_kg']) < .02
    hashes = {p.name:hashlib.sha256(p.read_bytes()).hexdigest()
              for p in [runtime/'python'/f'xgoduck_{p}.onnx' for p in ('walk','getup')]}
    return dict(conditions='20 s flat-floor deterministic run per configuration and start pose; neutral fixed jaw, '
                '0.2 m/s forward command, 90 degree fall poses, 50 Hz policy / 200 Hz physics.',
                policies_sha256=hashes,profile_sha256=hashlib.sha256(PROFILE.read_bytes()).hexdigest(),trials=results)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('profile','trials'))
    parser.add_argument('--runtime', type=Path, default=RUNTIME)
    args = parser.parse_args()
    if args.action == 'profile':
        PROFILE.write_text(json.dumps(generate(args.runtime),separators=(',',':'))+'\n',encoding='utf-8')
    else:
        result = trials(args.runtime, json.loads(PROFILE.read_text(encoding='utf-8')))
        (HERE/'motion-results.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
