"""Film paired stock-policy trials from recorded poses with true seated guard CAD."""
import argparse
from contextlib import ExitStack
import hashlib
import json
from pathlib import Path
import runpy

import mujoco as mj
import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = Path(__file__).resolve().parent
TEST = runpy.run_path(str(HERE / 'check-guard.py'))
CAD = runpy.run_path(str(HERE / 'rounded.py'))
CHECKS = TEST['checks']


def font(size, bold=False):
    for name in (('segoeuib.ttf', 'DejaVuSans-Bold.ttf') if bold else ('segoeui.ttf', 'DejaVuSans.ttf')):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            continue
    return ImageFont.load_default()


def record(profile, case, settle=None):
    frames = {}
    rigs = []

    def sample(rig, phase):
        slot = round(rig.data.time / .02)
        frames[slot] = (rig.data.qpos.copy(), phase)

    class RecordingRig(CHECKS['WalkSimulation']):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            rigs.append(self)

        def tick(self, *args, **kwargs):
            sample(self, self.recovery.phase)
            result = super().tick(*args, **kwargs)
            sample(self, self.recovery.phase)
            return result

    apply = TEST['configuration'](profile)
    if settle is None:
        def create(vx, yaw, **kwargs):
            kwargs['configure'] = apply
            return RecordingRig(vx, yaw, **kwargs)
        outcome = CHECKS['run_case'](case, [], {}, simulation_factory=create)
        outcome['pass'] = bool(CHECKS['passes'](case, outcome, TEST['STOCK'][case]))
    else:
        def observe_load(*args, **kwargs):
            policy, home, scale, feed = TEST['load_sim_policy'](*args, **kwargs)
            engine = policy.session

            class ObservedRoulade:
                def run(self, outputs, inputs):
                    # Read the exact pose supplied to each unmodified inference slot.
                    sample(rigs[0], 'roulade')
                    return engine.run(outputs, inputs)

            policy.session = ObservedRoulade()
            return policy, home, scale, feed
        outcome = TEST['roll'](RecordingRig, observe_load, apply, settle)
    assert sorted(frames) == list(range(max(frames) + 1))
    return outcome, frames, [j.name for j in (rigs[0].model.joint(i) for i in range(rigs[0].model.njnt))]


def verify(result, published):
    if result.keys() != published.keys():
        raise ValueError('Trial and replay outcome fields differ')
    for key, actual in result.items():
        expected = published[key]
        if isinstance(expected, float):
            if np.isclose(actual, expected, rtol=1e-8, atol=1e-8):
                continue
        elif actual == expected:
            continue
        raise ValueError(f'Replay diverged at {key}: expected {expected}, got {actual}')


def model(receiver, track, cover):
    from walk_sim import load_recovery_scene
    def show(spec):
        body = spec.body('trunk_base')
        for name, part, color in [('receiver', receiver, [.28,.36,.40,1]),
                                  ('rails', track, [.75,.59,.31,1]),
                                  ('cover', cover, [.22,.74,.65,1])]:
            mesh = part.simplify(.018).to_mesh()
            spec.add_mesh(name='render_'+name,
                          uservert=CAD['trunk'](mesh.vert_properties[:,:3]).ravel(),
                          userface=mesh.tri_verts.ravel())
            body.add_geom(name='render_'+name, type=mj.mjtGeom.mjGEOM_MESH, meshname='render_'+name,
                          mass=0, contype=0, conaffinity=0, group=1, rgba=color)
    return load_recovery_scene(mj, show)


class Panel:
    def __init__(self, m, stack):
        self.model = m
        self.data = mj.MjData(m)
        self.renderer = stack.enter_context(mj.Renderer(m, height=212, width=324))
        self.camera = mj.MjvCamera()
        self.trunk = m.body('trunk_base').id
        self.names = [m.joint(i).name for i in range(m.njnt)]

    def draw(self, qpos, case):
        self.data.qpos[:] = qpos
        mj.mj_forward(self.model, self.data)
        xyz = self.data.xpos[self.trunk]
        self.camera.lookat[:] = [xyz[0], xyz[1], max(.078,xyz[2]-.005)]
        self.camera.azimuth = 68 if case in ('right','left') else 82
        self.camera.elevation = -15
        self.camera.distance = .425
        self.renderer.update_scene(self.data, camera=self.camera)
        return Image.fromarray(self.renderer.render())


def status(row, phase, current, last, rolling):
    if current == last:
        if rolling:
            return f"{row['turn_deg']:.0f}° rotation   ·   " + ('roll passed' if row['pass_roll'] else 'roll missed gate')
        if row['pass']:
            return f"Up in {row['recovered_s']:.2f} s   ·   passed"
        return "Finished down" if not row['upright'] else "Finished upright   ·   missed gate"
    return {'default_pose':'Settling into fall', 'getup':'Get-up policy', 'walk':'Walking policy',
            'roulade':'Roll policy'}.get(phase, phase)


def film(panels, profiles, events, results, target, before_only=False):
    labels = ['Original guard', 'Original guard' if before_only else 'Rounded guard']
    typeface, titleface = font(14), font(16, True)
    animation, durations, verified = [], [], []
    for case, settle in events:
        rolling = settle is not None
        shots = []
        for side, panel, prof in zip(('before', 'after'), panels, profiles):
            row, samples, names = record(prof, case, settle)
            assert names == panel.names
            if results is not None:
                expected = results[side]['cases'][case] if not rolling else next(
                    r for r in results[side]['rolls'] if abs(r['settle_s']-settle)<1e-8)
                verify(row, expected)
            shots.append((row, samples))
        last = max(shots[0][1])
        assert last == max(shots[1][1])
        # Show every 100 ms until the 8 s gate, then explicitly jump to the final
        # 20 s state. Roll playback instead spans every phase, including walkoff.
        slots = list(range(0, last+1 if rolling else min(last,420)+1, 5))
        if last != slots[-1]:
            slots.append(last)
        title = (f'Forward roll   ·   launch at {settle:.2f} s' if rolling else
                 f'{case.capitalize()} fall   ·   90° release')
        for slot in slots:
            frame = Image.new('RGB', (652, 274), '#132128')
            draw = ImageDraw.Draw(frame)
            draw.text((7,3),title,fill='#eaf5f0',font=titleface)
            draw.text((643,5),f'{slot*.02:.1f} s',fill='#c1d6d1',font=typeface,anchor='ra')
            for i,(panel,(row,samples)) in enumerate(zip(panels,shots)):
                x=328*i
                q,phase=samples[slot]
                frame.paste(panel.draw(q,case),(x,28))
                draw.text((x+6,244),labels[i],font=typeface,fill='#e8f4ef')
                draw.text((x+318,245),status(row,phase,slot,last,rolling),font=typeface,fill='#a7c6bf',anchor='ra')
            animation.append(frame)
            durations.append(1300 if slot==last else 380 if slot==0 else 100)
        verified.append(dict(case=case,settle_s=settle,frames=len(slots),source_slots=last+1,
                             compared_to_trials=results is not None,
                             outcomes={side:shot[0] for side,shot in zip(('before','after'),shots)}))
    animation[0].save(target,format='WEBP',save_all=True,append_images=animation[1:],
                      duration=durations,loop=0,quality=64,method=3)
    with Image.open(target) as checked:
        assert checked.size==(652,274) and checked.info['loop']==0 and checked.n_frames>1
    return {'file':target.name,'bytes':target.stat().st_size,'events':verified}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--baseline-preview',action='store_true')
    p.add_argument('--output',type=Path,default=HERE/'media')
    p.add_argument('--groups',nargs='+',choices=['recovery','front-left','roulade'],
                   default=['recovery','front-left','roulade'])
    args=p.parse_args()
    if args.baseline_preview and args.output==HERE/'media':
        p.error('Use a separate preview output folder')
    args.output.mkdir(parents=True,exist_ok=True)
    plain_profile=json.loads((TEST['RAILS']/'plain-guard-profile.json').read_text())
    _,receiver,track,plain,*_=CAD['base']()
    if args.baseline_preview:
        after_profile,after_cover,results=plain_profile,plain,None
    else:
        info=json.loads((HERE/'fit.json').read_text())
        keys=('shoulder_depth_mm','shoulder_center_y_mm','shoulder_half_width_mm',
              'side_flare_mm','side_center_y_mm','side_center_z_mm')
        after_cover=CAD['rounded'](*(info[key] for key in keys))[0]
        after_profile=json.loads((HERE/'rounded-profile.json').read_text())
        results=json.loads((HERE/'trials.json').read_text())
        for key,digest in results['policies_sha256'].items():
            path=TEST['RUNTIME']/'python'/f'xgoduck_{key}.onnx'
            assert hashlib.sha256(path.read_bytes()).hexdigest()==digest
    groups={'recovery':[('back',None),('right',None)],'front-left':[('front',None),('left',None)],
            'roulade':[('roll',t) for t in (.2,.5,1.,2.)]}
    index=[]
    with ExitStack() as stack:
        panels=[Panel(model(receiver,track,cover),stack) for cover in (plain,after_cover)]
        for name in args.groups:
            item=film(panels,[plain_profile,after_profile],groups[name],results,args.output/f'guard-{name}.webp',
                      args.baseline_preview)
            print(name,item['bytes'],flush=True)
            index.append(item)
    (args.output/'replay-index.json').write_text(json.dumps(index,indent=2)+'\n',encoding='utf-8')


if __name__=='__main__':
    main()
