"""Compare head-sized passive tails on the V5 under the shipped ONNX policies."""
import argparse
import json
import runpy
from pathlib import Path

import mujoco as mj
import numpy as np

from tail_physics import HERE, configure, read_profile


PROFILE = read_profile()
ROOT = np.array(PROFILE['bodies']['proximal']['origin_m'])
# Fraction of tail length, lateral radius and vertical radius in metres.
# A slender root attaches to the tested yoke, then widens into a hollow shell.
SECTIONS = np.array([(0,.006,.006),(.18,.040,.025),(.42,.034,.022),
                     (.72,.021,.014),(1,.003,.003)])
AXIS = np.array([-np.cos(np.deg2rad(10)), 0, np.sin(np.deg2rad(10))])
UP = np.array([AXIS[2], 0, -AXIS[0]])
LEAN = np.deg2rad(-25)


def build(spec, length_mm, hinges, mass_factor, *, draped=False):
    if length_mm == 0:
        return
    configure(spec, PROFILE, 'locked')
    spec.delete(spec.body('tail_proximal'))
    parent = spec.body('tail_mount')
    length = length_mm / 1000
    links = max(1, hinges)
    angles = np.linspace(0, 2*np.pi, 24, endpoint=False)
    if draped and hinges!=1:
        raise ValueError('The curved root uses one passive hinge below the backpack')
    slopes=np.deg2rad([-63,-40,-18,3])
    sections=SECTIONS[:,0]
    axes=np.stack([-np.cos(slopes),np.zeros(4),np.sin(slopes)],axis=1)
    centers=np.vstack([np.zeros(3),np.cumsum(np.diff(sections)[:,None]*axes,axis=0)])
    def center(t):
        if not draped:
            return t*length*AXIS
        return length*np.array([np.interp(t,sections,c) for c in centers.T])

    bounds=[0,.18,1] if draped else np.linspace(0,1,links+1)
    last=0
    for link,(start,end) in enumerate(zip(bounds,bounds[1:])):
        body = parent.add_body(name=f'study_link_{link}', pos=ROOT if link==0 else center(start)-center(last))
        last=start
        articulated=bool(hinges and (not draped or link>0))
        if articulated:
            limit=8 if draped else 16
            body.add_joint(name=f'study_pitch_{link}', type=mj.mjtJoint.mjJNT_HINGE,
                           axis=[0,1,0], range=np.deg2rad([-limit,limit]), limited=True,
                           stiffness=0, damping=.00032, frictionloss=.00008, armature=0)
        cuts = np.unique([start, *SECTIONS[(SECTIONS[:,0]>start)&(SECTIONS[:,0]<end),0], end])
        # Exact volume integral of the elliptical frusta, using Simpson's rule.
        # 1.6 mm wall and 8% core fill. Joint hardware is budgeted separately.
        for band, (lo,hi) in enumerate(zip(cuts,cuts[1:])):
            stations = np.array([lo,(lo+hi)/2,hi])
            ry,rz = [np.interp(stations,SECTIONS[:,0],SECTIONS[:,i]) for i in (1,2)]
            full = np.pi*ry*rz
            hollow = np.pi*(ry-.0016)*(rz-.0016)
            mass = (hi-lo)*length*1240 * np.dot([1,4,1],full-.92*hollow)/6
            rings = []
            for i in [0,2]:
                if draped:
                    tangent_angle=np.interp(stations[i],sections,np.deg2rad([-63,-51.5,-29,-7.5,3]))
                    up=np.array([np.sin(tangent_angle),0,np.cos(tangent_angle)])
                else:
                    up=UP
                ring = (center(stations[i])-center(start) + np.outer(np.sin(angles)*rz[i],up)
                        + np.outer(np.cos(angles)*ry[i],[0,1,0]))
                rings.extend(ring)
            name = f'candidate_{link}_{band}'
            spec.add_mesh(name=name,uservert=np.ravel(rings).tolist())
            body.add_geom(name=name,type=mj.mjtGeom.mjGEOM_MESH,meshname=name,
                          mass=mass*mass_factor,contype=1,conaffinity=0,group=1,
                          rgba=[.32+.08*(link%2),.66,.52,1])
        if draped and end==1:
            body.add_site(name='tail_tip',pos=center(1)-center(start),size=[.001,0,0])
        if articulated:
            body.add_geom(type=mj.mjtGeom.mjGEOM_SPHERE,size=[.003,0,0],
                          mass=.0008*mass_factor,contype=0,conaffinity=0,group=1)
        parent = body


def head_measurements(sim):
    m,d = sim.model,sim.data
    jaw = m.body('jaw_soft').id
    geom = next(g for g in range(m.ngeom) if m.geom_bodyid[g]==jaw and m.geom_dataid[g]>=0)
    mesh = m.geom_dataid[geom]
    start,n = m.mesh_vertadr[mesh],m.mesh_vertnum[mesh]
    xyz = m.mesh_vert[start:start+n] @ d.geom_xmat[geom].reshape(3,3).T
    neck = m.body('neck').id
    x0 = d.subtree_com[neck,0]
    q = m.joint('neck_pitch').qposadr[0]
    home = d.qpos[q]
    d.qpos[q] += LEAN
    mj.mj_forward(m,d)
    dx = d.subtree_com[neck,0]-x0
    d.qpos[q] = home
    mj.mj_forward(m,d)
    return dict(head_mesh_span_mm=(np.ptp(xyz,axis=0)*1000).round(2).tolist(),
                neck_subtree_g=round(m.body_subtreemass[neck]*1000,3),
                forward_shift_mm=round(dx*1000,3),
                extra_moment_Nm=round(dx*m.body_subtreemass[neck]*9.81,6),
                stand_foot_axis_x_m=float(np.mean([d.site_xpos[m.site(s+'_foot').id,0]
                                                    for s in ('left','right')])))


def run_trial(checks, case, length, hinges, factor, head, fall_delta=0, lean_deg=-25,
              floor_contacts='all'):
    samples = []
    def configure_trial(spec):
        build(spec,length,hinges,factor)
        for geom in spec.geoms:
            name=geom.name or ''
            if ((floor_contacts in ('none','links') and name.startswith('tail_mount_')) or
                (floor_contacts in ('none','mount') and name.startswith('candidate_'))):
                geom.contype=0
    class StudySimulation(checks['WalkSimulation']):
        def __init__(self,vx,yaw,**kwargs):
            kwargs['configure'] = configure_trial
            super().__init__(vx,yaw,**kwargs)
            m = self.model
            self.extra = [b for b in range(m.nbody) if m.body(b).name.startswith(('study_','tail_'))]

        def tick(self):
            d,m = self.data,self.model
            if case=='right' and d.time==0 and fall_delta:
                mj.mju_axisAngle2Quat(d.qpos[self.root+3:self.root+7],
                                      np.array([1.,0,0]),np.deg2rad(90+fall_delta))
            # Recovery uses its own nominal head pose. Lean walking reuses the
            # stock policy's head-command observation without forcing servo poses.
            if case=='walk':
                self.obs[0,51]=np.deg2rad(lean_deg)*np.clip((d.time-2)/1.5,0,1)
            result=super().tick()
            if case=='walk' and d.time>=5:
                mj.mj_forward(m,d)
                # Transform the fixed standing support reference with trunk yaw
                # only. Horizontal moment arms include the actual trunk pitch.
                rot=d.xmat[self.trunk].reshape(3,3)
                fwd=rot[:2,0]/np.linalg.norm(rot[:2,0])
                x=(d.xipos[:,:2]-d.xpos[self.trunk,:2]) @ fwd
                torque=-9.81*np.sum(m.body_mass[self.extra]*(x[self.extra]-head['stand_foot_axis_x_m']))
                neck=m.body('neck').id
                head_x=(d.subtree_com[neck]-d.xpos[self.trunk]) @ rot[:,0]
                samples.append((torque,float(head_x),np.rad2deg(d.qpos[self.qpos[5]])))
            return result

    row=checks['run_case'](case,[],{},simulation_factory=StudySimulation)
    if samples:
        torque,head_x,neck=np.mean(samples,axis=0)
        row.update(rearward_moment_Nm=round(torque,5),mean_head_x_mm=round(1000*head_x,2),
                   actual_neck_deg=round(neck,2),offset_percent=round(100*torque/head['extra_moment_Nm'],1))
    return row


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('runtime',type=Path)
    p.add_argument('output',type=Path)
    args=p.parse_args()
    checks=runpy.run_path(str(args.runtime/'tools/check-duck-hardware.py'))
    head=head_measurements(checks['WalkSimulation'](.2,0))
    report=dict(head=head, cases=[], sensitivity=[])
    # Fractions of the measured head's fore-aft length, rounded to the nearest mm.
    lengths=[round(head['head_mesh_span_mm'][0]*r) for r in (1,1.06,1.25,1.5)]
    for length,hinges in [(0,0),*((l,j) for l in lengths for j in (0,1,2,3))]:
        row=dict(length_mm=length,hinges=hinges)
        for case in ('walk','right'):
            trial=run_trial(checks,case,length,hinges,1.,head)
            ref=report['cases'][0][case] if report['cases'] else trial
            trial['pass']=bool(checks['passes'](case,trial,ref))
            row[case]=trial
        report['cases'].append(row)
        print(length,hinges,round(row['walk']['forward_m'],2),row['walk']['offset_percent'],row['right']['pass'],flush=True)
    # Compare the simplest articulated design to the closest moment match.
    qualified=[r for r in report['cases'][1:] if r['walk']['pass'] and r['right']['pass']]
    walked=[r for r in report['cases'][1:] if r['walk']['pass']]
    ranked=sorted(qualified or walked or report['cases'][1:],
                     key=lambda r: abs(100-r['walk']['offset_percent']))[:2]
    simple=min((r for r in (qualified or walked or report['cases'][1:]) if r['hinges']==1),
               key=lambda r:abs(100-r['walk']['offset_percent']),default=ranked[0])
    finalists=[simple,next(r for r in ranked if r is not simple)]
    for r in [report['cases'][0],*finalists]:
        r['neutral_walk']=run_trial(checks,'walk',r['length_mm'],r['hinges'],1.,head,lean_deg=0)
    for r in finalists:
        for factor in (.8,1.2):
            for case in ('walk','right'):
                v=run_trial(checks,case,r['length_mm'],r['hinges'],factor,head)
                report['sensitivity'].append(dict(length_mm=r['length_mm'], hinges=r['hinges'],
                     mass_factor=factor,case=case,delta_deg=0,trial=v,
                     passed=bool(checks['passes'](case,v,report['cases'][0][case]))))
        for delta in (-5,5):
            v=run_trial(checks,'right',r['length_mm'],r['hinges'],1.,head,fall_delta=delta)
            report['sensitivity'].append(dict(length_mm=r['length_mm'],hinges=r['hinges'],
                mass_factor=1,case='right',delta_deg=delta,trial=v,
                passed=bool(checks['passes']('right',v,report['cases'][0]['right']))))
    # Hold the entire backpack mass constant across lengths. This isolates the
    # lever arm from the extra plastic in a longer constant-wall prototype.
    stock=report['cases'][0]
    mount_g=1000*PROFILE['bodies']['mount']['mass_kg']
    reference=next(r for r in report['cases'] if r['length_mm']==lengths[1] and r['hinges']==1)
    fixed_g=stock['walk']['mass_g']+mount_g
    moving_g=reference['walk']['mass_g']-fixed_g
    report['matched_mass']=[]
    for r in report['cases'][1:]:
        if r['hinges']!=1 or r is reference:
            continue
        factor=moving_g/(r['walk']['mass_g']-fixed_g)
        row=dict(length_mm=r['length_mm'],hinges=1,mass_factor=factor)
        for case in ('walk','right'):
            v=run_trial(checks,case,r['length_mm'],1,factor,head)
            v['pass']=bool(checks['passes'](case,v,stock[case]))
            row[case]=v
        report['matched_mass'].append(row)
    report['right_contact_controls']={}
    for contacts in ('mount','links','none'):
        v=run_trial(checks,'right',reference['length_mm'],1,1.,head,floor_contacts=contacts)
        v['pass']=bool(checks['passes']('right',v,stock['right']))
        report['right_contact_controls'][contacts]=v
    args.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8')


if __name__=='__main__':
    main()
