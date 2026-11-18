"""Screen the actual shoulder CAD and stock ONNX policies against synthetic stationary panel geometry."""
import argparse
import json
import runpy
import sys
from functools import lru_cache
from pathlib import Path

import mujoco as mj
import numpy as np

HERE=Path(__file__).resolve().parent
TAIL=HERE.parent/'v5-raptor-tail'
sys.path.insert(0,str(TAIL))
from tail_physics import collider_vertices, configure, mass_properties, read_profile, triangles, trunk

CAD=runpy.run_path(str(HERE/'shoulder-pointer.py'))
STACK=runpy.run_path(str(HERE/'stack/stack.py'))


@lru_cache
def profile(pose):
    deck,cart,pin=STACK['deck'](),STACK['cartridge'](),STACK['pin']()
    groups=[('deck',deck,1240)]
    if pose!='removed':
        parts=CAD['shoulder_parts'](pose)
        groups.extend([('cartridge',cart+parts['stem'],1240),('pin',pin,1240)])
        groups.extend((name,s,1200) for name,s in parts.items() if name!='stem')
        for sign in [-1,1]:
            pivot=CAD['HUB']+[0,sign*5.5,0]
            # M2 small screw/nut pair represented by a 0.30 g lump at the hinge.
            solid=CAD['cyl'](pivot-[0,0,2.8],1.8,10)
            groups.append(('hardware'+str(sign),CAD['seated'](solid),300000/solid.volume()))
    meshes=[trunk(triangles(s)) for _,s,_ in groups]
    densities=np.concatenate([np.full(len(m),d) for m,(_,_,d) in zip(meshes,groups)])
    mass,com,inertia=mass_properties(np.concatenate(meshes),densities)
    # Split the platform, so contact hulls do not fill the space above the board.
    hulls=collider_vertices(deck,np.zeros(3),'mount')
    if pose!='removed':
        hulls+=collider_vertices(cart,np.zeros(3),'mount')
    return mass,com,inertia,hulls


def add_stack(spec,pose,tail=False):
    if pose=='bare':
        return
    if tail:
        roll=runpy.run_path(str(TAIL/'recovery-shapes.py'))
        roll['build'](spec,length_mm=167,width_factor=.90,shoe=[0,25,-69,4.5],lower_anchor_z=-66.5)
    else:
        configure(spec,read_profile(),'locked')
        spec.delete(spec.body('tail_proximal'))
    mass,com,inertia,hulls=profile(pose)
    body=spec.body('trunk_base').add_body(name='shoulder_stack')
    body.explicitinertial=True
    body.mass=mass
    body.ipos=com
    body.fullinertia=inertia[[0,1,2,0,0,1],[0,1,2,1,2,2]]
    for i,vertices in enumerate(hulls):
        name=f'stack_{i}'
        spec.add_mesh(name=name,uservert=np.ravel(vertices))
        body.add_geom(name=name,type=mj.mjtGeom.mjGEOM_MESH,meshname=name,contype=1,conaffinity=0,group=1)
    if pose=='removed':
        return
    for i,(kind,p,q,r) in enumerate(CAD['segments'](pose)):
        body.add_geom(name=f'feeler_{kind}_{i}',type=mj.mjtGeom.mjGEOM_CAPSULE,
                      fromto=np.r_[p,q]*.001,size=[r*.001,0,0],contype=5,conaffinity=0,group=1)
    body.add_geom(name='pointer_pad',type=mj.mjtGeom.mjGEOM_SPHERE,
                  pos=CAD['TOUCH']*.001,size=[.0036,0,0],contype=5,conaffinity=0,group=1)
    for sign in [-1,1]:
        p=CAD['HUB']+[0,sign*5.5,1.6]
        body.add_geom(name='feeler_pivot_'+str(sign),type=mj.mjtGeom.mjGEOM_CYLINDER,
                      pos=p*.001,size=[.00365,.0045,0],contype=5,conaffinity=0,group=1)
    # All registered stock links and the floor can collide with this stiff tool.
    for g in spec.geoms:
        if g.meshname and g.contype and not g.name.startswith(('stack_','tail_','candidate_','feeler_')):
            g.conaffinity|=4


def sight(rig, yaw, pitch, vfov=45):
    m,d=rig.model,rig.data
    for name,angle in [('neck_pitch',20),('head_pitch',pitch),('head_yaw',yaw)]:
        d.qpos[m.joint(name).qposadr]=np.deg2rad(angle)
    mj.mj_forward(m,d)
    tip=d.geom_xpos[m.geom('pointer_pad').id]
    eye=d.site_xpos[m.site('head_camera').id]
    cam=d.xmat[m.body('jaw_soft').id].reshape(3,3)
    in_view=(tip-eye)@cam
    depth=-in_view[2]
    focal=240/np.tan(np.deg2rad(vfov/2))
    uv=np.array([320+focal*in_view[0]/depth,240-focal*in_view[1]/depth])
    vec=tip-eye
    hit=np.array([-1],dtype=np.int32)
    mj.mj_ray(m,d,eye,vec/np.linalg.norm(vec),None,1,-1,hit)
    first=m.geom(int(hit[0])).name if hit[0]>=0 else ''
    # Check actual physical meshes against the conservative arm envelope.
    head=[g for g in range(m.ngeom) if m.geom_dataid[g]>=0 and
          m.body(m.geom_bodyid[g]).name in ('neck','neck_pitch','yaw_roll_motion','jaw_soft') and m.geom_contype[g]]
    tool=[g for g in range(m.ngeom) if m.geom(g).name.startswith(('feeler_','pointer_'))]
    clearance=min(mj.mj_geomDistance(m,d,h,t,.15,None) for h in head for t in tool)
    ahead=d.xmat[rig.trunk].reshape(3,3)[:,0]
    # Check if any stock surface, anywhere on an infinite upright panel, leads the tip.
    furthest=-np.inf
    for g in range(m.ngeom):
        mesh=m.geom_dataid[g]
        if m.geom_type[g]!=mj.mjtGeom.mjGEOM_MESH or mesh<0 or not m.geom_contype[g]:
            continue
        if m.geom(g).name.startswith(('stack_','tail_')):
            continue
        v=m.mesh_vert[m.mesh_vertadr[mesh]:m.mesh_vertadr[mesh]+m.mesh_vertnum[mesh]]
        x=d.geom_xpos[g]@ahead+np.max(v@(d.geom_xmat[g].reshape(3,3).T@ahead))
        furthest=max(furthest,x)
    # The stem can hide the exact centre from behind. Sample the projected
    # pad footprint to determine whether the exposed soft pad is identifiable.
    pad_samples=0
    if depth>0 and -20<uv[0]<660 and -20<uv[1]<500:
        extent=int(np.ceil(.0036/depth*focal))+2
        for u in range(max(0,int(uv[0])-extent),min(640,int(uv[0])+extent+1),4):
            for v in range(max(0,int(uv[1])-extent),min(480,int(uv[1])+extent+1),4):
                ray=cam@np.array([(u-320)/focal,(240-v)/focal,-1.])
                hit[0]=-1
                mj.mj_ray(m,d,eye,ray/np.linalg.norm(ray),None,1,-1,hit)
                pad_samples+=int(hit[0]==m.geom('pointer_pad').id)
    return dict(head_yaw_deg=yaw,head_pitch_deg=pitch,vertical_fov_deg=vfov,uv_px=uv.round(1).tolist(),
                depth_mm=round(depth*1000,2),first_ray_surface=first,
                pad_visible_area_px_est=pad_samples*16,head_clearance_mm=round(clearance*1000,2),
                tip_lead_over_stock_at_panel_mm=round((tip@ahead+.0036-furthest)*1000,2))


def camera_checks(factory):
    rig=factory(0,0,configure=lambda s:add_stack(s,'closed'))
    rows=[sight(rig,y,p) for y in range(-35,36,5) for p in range(-10,56,5)]
    valid=[r for r in rows if r['depth_mm']>0 and r['pad_visible_area_px_est']>=48 and r['head_clearance_mm']>=3
           and r['tip_lead_over_stock_at_panel_mm']>3 and 20<r['uv_px'][0]<620 and 20<r['uv_px'][1]<460]
    if not valid:
        print('Closest',[(r['head_yaw_deg'],r['head_pitch_deg'],r['uv_px'],r['first_ray_surface']) for r in rows[::22]])
        raise ValueError('No usable stationary aiming pose on grid')
    best=min(valid,key=lambda r:np.linalg.norm(np.array(r['uv_px'])-[320,260]))
    steps=[sight(rig,best['head_yaw_deg']*f,20+(best['head_pitch_deg']-20)*f) for f in np.linspace(0,1,31)]
    # Move the cassette geometry only. Deck blockers are checked separately in stack.py.
    m,d=rig.model,rig.data
    tools=[g for g in range(m.ngeom) if m.geom(g).name.startswith(('feeler_','pointer_'))]
    stock=[g for g in range(m.ngeom) if m.geom_dataid[g]>=0 and m.geom_contype[g] and
           m.body(m.geom_bodyid[g]).name in ('neck','neck_pitch','yaw_roll_motion','jaw_soft','trunk_base')]
    origin=m.geom_pos[tools].copy()
    extraction={}
    for name,angle_yaw,angle_pitch in [('neutral',0,20),('aimed',best['head_yaw_deg'],best['head_pitch_deg'])]:
        d.qpos[m.joint('head_yaw').qposadr]=np.deg2rad(angle_yaw)
        d.qpos[m.joint('head_pitch').qposadr]=np.deg2rad(angle_pitch)
        gaps=[]
        for distance in np.linspace(0,.030,31):
            m.geom_pos[tools]=origin-[distance,0,0]
            mj.mj_forward(m,d)
            gaps.extend(mj.mj_geomDistance(m,d,h,t,.15,None) for h in stock for t in tools)
        extraction[name]=round(1000*min(gaps),2)
    m.geom_pos[tools]=origin
    return dict(best=best,accepted=len(valid),total=len(rows),neutral_to_aim_min_clearance_mm=min(x['head_clearance_mm'] for x in steps),
                withdrawal_30mm_min_stock_gap_mm=extraction,
                alternative_fields=[sight(rig,best['head_yaw_deg'],best['head_pitch_deg'],f) for f in [35,55]],
                synthetic_press=press(rig))


def press(rig):
    """Move the trunk-mounted tool 2 mm into a fictitious vertical key with stationary soles."""
    from scipy.spatial import ConvexHull
    m,d=rig.model,rig.data
    axes=[m.joint(side+'_'+j) for side in ('left','right') for j in ('hip_yaw','hip_roll','hip_pitch','knee','ankle')]
    dofs=np.array([j.dofadr[0] for j in axes])
    qs=np.array([j.qposadr[0] for j in axes])
    q0=d.qpos.copy()
    sites=[m.site(s+'_foot').id for s in ('left','right')]
    feet0=d.site_xpos[sites].copy()
    rotations=d.site_xmat[sites].reshape(2,3,3).copy()
    tip=m.geom('pointer_pad').id
    tip0=d.geom_xpos[tip].copy()
    support=[]
    for side in ['left','right']:
        g=m.geom(side+'_foot_collision').id
        mesh=m.geom_dataid[g]
        v=m.mesh_vert[m.mesh_vertadr[mesh]:m.mesh_vertadr[mesh]+m.mesh_vertnum[mesh]]
        v=v@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g]
        support.extend(v[v[:,2]<np.min(v[:,2])+.001,:2])
    hull=ConvexHull(support).equations
    margins=[]
    heading=d.xmat[rig.trunk].reshape(3,3)[:,0].copy()
    for stroke in np.linspace(0,.002,11):
        d.qpos[rig.root:rig.root+3]=q0[rig.root:rig.root+3]+heading*stroke
        for _ in range(6):
            mj.mj_forward(m,d)
            residual=[]
            jac=[]
            for i,s in enumerate(sites):
                rot=d.site_xmat[s].reshape(3,3)
                delta=.5*np.cross(rot.T,rotations[i].T).sum(axis=0)
                residual.extend([*(feet0[i]-d.site_xpos[s]),*(.05*delta)])
                pos=np.zeros((3,m.nv));ang=np.zeros_like(pos)
                mj.mj_jacSite(m,d,pos,ang,s)
                jac.extend([*pos[:,dofs],*(.05*ang[:,dofs])])
            if np.linalg.norm(residual)<1e-10: break
            d.qpos[qs]+=np.linalg.lstsq(jac,residual,rcond=1e-6)[0]
        margins.append(np.min(-(hull[:,:2]@d.subtree_com[rig.trunk,:2]+hull[:,2])))
    joint_change=np.max(np.abs(np.rad2deg(d.qpos[qs]-q0[qs])))
    return dict(tip_advance_mm=round((d.geom_xpos[tip]-tip0)@heading*1000,4),
                max_end_sole_translation_error_mm=round(float(np.max(np.linalg.norm(d.site_xpos[sites]-feet0,axis=1)))*1000,6),
                largest_leg_joint_change_deg=round(float(joint_change),3),
                min_quasistatic_com_support_margin_mm=round(float(min(margins))*1000,2))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('runtime',type=Path)
    p.add_argument('output',type=Path)
    args=p.parse_args()
    checks=runpy.run_path(str(args.runtime/'tools/check-duck-hardware.py'))
    camera=camera_checks(checks['WalkSimulation'])
    print('camera',camera['best'],flush=True)
    runs=[]
    for tail,pose in [(False,'bare'),(False,'removed'),(False,'closed'),(False,'open'),
                      (True,'removed'),(True,'closed'),(True,'open')]:
        def rig(vx,yaw,**kwargs):
            kwargs['configure']=lambda s:add_stack(s,pose,tail)
            return checks['WalkSimulation'](vx,yaw,**kwargs)
        cases=('walk','right','left','front','back') if not tail else ('walk','right')
        for case in cases:
            result=checks['run_case'](case,[],{},simulation_factory=rig)
            stock=next((r['outcome'] for r in runs if r['pose']=='bare' and r['case']==case),result)
            passed=bool(checks['passes'](case,result,stock))
            runs.append(dict(tail=tail,pose=pose,case=case,passed=passed,outcome=result))
            print(tail,pose,case,passed,result['recovered_s'],round(result['forward_m'],2),flush=True)
    lean=[]
    for pose in ('removed','closed','open'):
        mod=runpy.run_path(str(TAIL/'posture-study.py'))
        cls=mod['simulation'](checks['WalkSimulation'],167,12,-48,-65,True,.45,
            build_attachment=lambda spec:add_stack(spec,pose,True),transition_after_recovery=True)
        outcome=checks['run_case']('right',[],{},simulation_factory=cls)
        lean.append(dict(pose=pose,passed=bool(checks['passes']('right',outcome,outcome)),outcome=outcome))
    report=dict(camera=camera,stack_plus_pointer_g={pose:round(profile(pose)[0]*1000,3) for pose in ['removed','closed','open']},
                runs=runs,tail_right_recovery_then_moderate_lean=lean)
    args.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8')


if __name__=='__main__':
    main()
