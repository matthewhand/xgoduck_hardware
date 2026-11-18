"""Replay stock policies with equal-mass locked and passive V5 tails."""
import argparse
import hashlib
import json
from pathlib import Path
import runpy
import sys

import numpy as np
from tail_physics import HERE, PITCH, YAW, VARIANTS, configure, read_profile


class FlightMeter:
    """Count unsupported 5 ms physics samples between alternating foot contacts."""
    def __init__(self):
        self.last_foot = None
        self.air = 0
        self.flights = []
        self.samples = self.unsupported = self.tail_contact = 0
        self.named_geoms = None

    def sample(self, sim, feet):
        d = sim.data
        if d.time < 2.0:
            return
        if self.named_geoms is None:
            self.named_geoms = {i for i in range(sim.model.ngeom)
                                if (sim.model.geom(i).name or '').startswith('tail_')}
        floor_geoms = set()
        for c in d.contact:
            a,b = c.geom
            if a == sim.floor:
                floor_geoms.add(b)
            elif b == sim.floor:
                floor_geoms.add(a)
        self.samples += 1
        self.tail_contact += bool(floor_geoms & self.named_geoms)
        if not floor_geoms and sim.recovery.phase=='walk':
            self.air += 1
            self.unsupported += 1
            return
        single = next(iter(feet)) if len(feet)==1 and floor_geoms<=sim.feet.keys() else None
        if self.air and self.last_foot and single and single!=self.last_foot:
            self.flights.append(round(self.air * sim.model.opt.timestep,4))
        self.air = 0
        self.last_foot = single

    def result(self):
        effective = [t for t in self.flights if t>=.020]
        return dict(alternating_flights_20ms=len(effective),max_alternating_flight_s=max(self.flights,default=0),
                    unsupported_fraction=round(self.unsupported/max(1,self.samples),4),
                    tail_floor_fraction=round(self.tail_contact/max(1,self.samples),4))


def run(runtime, output, recording, walk_policy=None):
    runtime = runtime.resolve()
    checks = runpy.run_path(str(runtime/'tools/check-duck-hardware.py'))
    from walk_sim import JOINT_NAMES, WalkSimulation
    profile = read_profile()
    speeds = {'fast-forward': .45}
    cases = ('walk','turn-left','turn-right','back','front','left','right','fast-forward')
    results = {}
    for variant in VARIANTS:
        rows = {}
        for case in cases:
            meter = FlightMeter()
            frames = []
            hinges = []
            record = variant=='passive' and case=='walk'
            class Simulation(WalkSimulation):
                def __init__(self,vx,yaw,**kwargs):
                    super().__init__(speeds.get(case,vx),yaw,**dict(kwargs,configure=lambda s:configure(s,profile,variant)))
                    if variant=='passive':
                        hinges.extend(int(self.model.joint(n).qposadr[0]) for n in (PITCH,YAW))
                    self.minimum = [float('inf')]*len(hinges)
                    self.maximum = [-float('inf')]*len(hinges)
                    self.slot = 0
                def tick(self):
                    value = super().tick(observe_contacts=meter.sample)
                    angles = self.data.qpos[hinges]
                    self.minimum = np.minimum(self.minimum,angles)
                    self.maximum = np.maximum(self.maximum,angles)
                    self.slot += 1
                    if record and 100<=self.slot<=300 and self.slot%2==0:
                        frames.append(dict(t=round(self.data.time-2,3),
                            root=self.data.qpos[self.root:self.root+7].round(7).tolist(),
                            joints=self.data.qpos[self.qpos].round(6).tolist(), hinges=angles.round(6).tolist()))
                    return value
            sims=[]
            def create(*args,**kwargs):
                sim=Simulation(*args,**kwargs)
                sims.append(sim)
                return sim
            policies = {'walk_file':walk_policy.resolve()} if walk_policy else {}
            tr = checks['run_case'](case if case!='fast-forward' else 'walk',[],policies,simulation_factory=create)
            tr.update(meter.result())
            if case=='fast-forward':
                # A conservative study gate: enough speed and repeated unsupported
                # transitions alternating feet while the full robot stays upright.
                tr['mean_speed_m_s']=tr['forward_m']/20
                tr['running_gate']=bool(tr['upright'] and not tr['fell'] and tr['mean_speed_m_s']>=.35
                                        and tr['alternating_flights_20ms']>=8)
            else:
                reference = tr if variant=='bare' else results['bare'][case]
                tr['pass'] = bool(checks['passes'](case,tr,reference))
            if hinges:
                tr['hinge_deg_range']=np.rad2deg(np.stack((sims[0].minimum,sims[0].maximum),axis=1)).round(2).tolist()
            rows[case]=tr
            if record:
                controller=walk_policy.name if walk_policy else 'Stock walk ONNX'
                trace=dict(version=1,variant=variant,case=case,joint_names=list(JOINT_NAMES),hinge_names=[PITCH,YAW],
                           controller=controller+'; two passive, friction-damped hinges, no tail motor or spring.',
                           time_offset_s=2,sample_hz=25, frames=frames)
                recording.write_text(json.dumps(trace,separators=(',',':'))+'\n',encoding='utf-8')
            print(variant,case,tr.get('pass',tr.get('running_gate')),round(tr['forward_m'],2),flush=True)
        results[variant]=rows
    policy_dir = runtime/'python'
    files = (walk_policy or policy_dir/'xgoduck_walk.onnx',policy_dir/'xgoduck_getup.onnx')
    hashes = {file.name:hashlib.sha256(file.read_bytes()).hexdigest() for file in files}
    output.write_text(json.dumps(dict(conditions='Nominal 20 s deterministic trials, 50 Hz policy, 200 Hz physics. No other mods.',
        forward_commands_m_s={'walk':.2,'fast-forward':.45},physics_profile_sha256=hashlib.sha256((HERE/'tail-physics.json').read_bytes()).hexdigest(),
        policies=hashes,trials=results),indent=2,allow_nan=False)+'\n',encoding='utf-8')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('runtime',type=Path)
    p.add_argument('report',type=Path)
    p.add_argument('recording',type=Path)
    p.add_argument('--walk-policy',type=Path,help='Evaluate an exported candidate with the runtime policy metadata checks')
    a=p.parse_args()
    run(a.runtime,a.report,a.recording,a.walk_policy)
