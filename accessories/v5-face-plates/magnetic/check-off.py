"""Qualify the bare concealed receivers and the same setup wearing the everyday guard."""
import argparse
import hashlib
import json
from pathlib import Path
import runpy
import sys

import numpy as np

HERE=Path(__file__).resolve().parent
ACC=HERE.parents[1]
TAIL=ACC/'v5-raptor-tail'
RAILS=ACC/'v5-slide-on-backpack'/'removable-rails'
RUNTIME=ACC.parents[1]/'xgoduck_runtime_arduino'
PROFILE=HERE/'off-head-profile.json'
sys.path.insert(0,str(TAIL))


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def head_profile(runtime):
    from tail_physics import triangles,mass_properties
    from scipy.spatial import ConvexHull
    beak=ACC/'v5-dino-beak'
    preview=runpy.run_path(str(beak/'export-preview.py'))
    list(preview['stock_head'](runtime))
    screw=runpy.run_path(str(beak/'screw-teeth'/'screw-teeth.py'))
    chin=runpy.run_path(str(HERE/'chin.py'))
    brow=runpy.run_path(str(HERE/'brow.py'))
    teeth=screw['build']()
    lower=chin['build'](teeth['lower']['mod'])
    pairs=[('face','upper',brow['CAD']['read_stock']('FACE'),brow['magnetic_face']()),
           ('head_bottom','upper',teeth['upper']['stock'],teeth['upper']['mod']),
           ('jaw','lower',teeth['lower']['stock'],lower['host'])]
    face_to_upper=((0,-1,0,0),(0,0,1,brow['BASE']['FACE_Y']),(-1,0,0,33.258))
    solids=[]
    hull_errors={}
    for name,side,old,new in pairs:
        # Only voids are created. A curved cheek loses a few mesh vertices at the tiny
        # access holes. Measure each old extreme point's distance to the new convex hull.
        assert (new-old).volume()<1e-4
        vertices=old.to_mesh().vert_properties[:,:3]
        original=vertices[ConvexHull(vertices).vertices]
        retained=new.to_mesh().vert_properties[:,:3].astype(float)
        new_hull=ConvexHull(retained)
        a,b,c=retained[new_hull.simplices].transpose(1,0,2)
        normal=np.cross(b-a,c-a)
        normal/=np.linalg.norm(normal,axis=1)[:,None]
        error=0.
        gaps=original@new_hull.equations[:,:3].T+new_hull.equations[:,3]
        for point in original[np.max(gaps,axis=1)>1e-9]:
            height=np.einsum('ij,ij->i',point-a,normal)
            projected=point-height[:,None]*normal
            inside=np.ones(len(a),dtype=bool)
            edges=[]
            for start,end in [(a,b),(b,c),(c,a)]:
                line=end-start
                inside &= (np.einsum('ij,ij->i',np.cross(line,projected-start),normal)>=-1e-10)
                t=np.einsum('ij,ij->i',point-start,line)/np.einsum('ij,ij->i',line,line)
                edges.append(np.linalg.norm(point-(start+np.clip(t,0,1)[:,None]*line),axis=1))
            error=max(error,float(min(np.min(np.abs(height),where=inside,initial=np.inf),np.min(edges))))
        assert error<.01,(name,error)
        hull_errors[name]=error
        removed=old-new
        if name=='face':
            removed=removed.transform(face_to_upper)
        solids.append((name+'_plastic_removed',side,removed,1200,-1))
    def include(name,side,item,host,density):
        assert (item-host).volume()<1e-4,name
        solids.append((name,side,item,density,1))
    for label,item,density in [('face_target_keepers',brow['rear_keepers'](),1200),
                                ('face_steel_targets',brow['target_discs'](),7850)]:
        p=brow['to_print'](item)
        assert (p-pairs[0][2]).volume()<1e-4
        solids.append((label,'upper',p.transform(face_to_upper),density,1))
    include('chin_keeper','lower',lower['lid'],pairs[2][2],1200)
    include('chin_steel_targets','lower',lower['steel'],pairs[2][2],7850)
    for side in ('upper','lower'):
        # Hardware alternates between screw and square nut for one side in rail().
        nuts=screw['pair'](screw['union'](teeth[side]['hardware'][1::2]))
        assert len(nuts.decompose())==4
        include(side+'_four_square_nuts',side,nuts,teeth[side]['stock'],7850)
    parts={}
    for name,side,solid,density,sign in solids:
        v=preview['registered'](triangles(solid),side)/1000
        mass,com,inertia=mass_properties(v,np.full(len(v),density))
        assert abs(mass-density*solid.volume()*1e-9)<1e-8
        parts[name]=dict(sign=sign,mass_kg=mass,com_m=com.tolist(),inertia_kg_m2=inertia.tolist())
    return dict(parent='jaw_soft',source_head_sha256=preview['HEAD_SHA'],
                max_stock_convex_vertex_displacement_mm=hull_errors,
                delta_mass_g=sum(1000*p['sign']*p['mass_kg'] for p in parts.values()),parts=parts)


def run(runtime):
    from rail_only import configure_rails
    rig=runpy.run_path(str(runtime/'tools/check-duck-hardware.py'))
    from walk_sim import load_sim_policy
    roll=runpy.run_path(str(TAIL/'roll-after-removal.py'))['roll']
    r=runpy.run_path(str(RAILS/'receiver_physics.py'))
    header=json.loads(PROFILE.read_text())
    guard=json.loads((RAILS/'plain-guard-profile.json').read_text())
    rear=json.loads((RAILS/'receiver-profile.json').read_text())
    def stripped(spec):
        r['configure_receiver'](spec)
        body=spec.body(header['parent'])
        for piece in header['parts'].values():
            # subtract() performs the parallel-axis translation with the signed mass.
            s=-piece['sign']
            r['subtract'](body,s*piece['mass_kg'],piece['com_m'],s*np.array(piece['inertia_kg_m2']))
    def everyday(spec):
        stripped(spec)
        configure_rails(spec,guard)
    results={}
    for title,config in [('stock',None),('receivers_only',stripped),('everyday_guard',everyday)]:
        cases={}
        def create(vx,yaw,**kwargs):
            kwargs['configure']=config
            return rig['WalkSimulation'](vx,yaw,**kwargs)
        for case in ('walk','front','back','left','right'):
            row=rig['run_case'](case,[],{},simulation_factory=create)
            ref=row if title=='stock' else results['stock']['policy'][case]
            row['passed']=bool(rig['passes'](case,row,ref))
            cases[case]=row
            print(title,case,row['passed'],row['recovered_s'],flush=True)
        rolls=[roll(rig['WalkSimulation'],load_sim_policy,config,t) for t in (.2,.5,1.,2.)]
        results[title]=dict(policy=cases,roulade=rolls)
        print(title,'rolls',[q['turn_deg'] for q in rolls],flush=True)
        stock_mass=results['stock']['roulade'][0]['robot_mass_g']
        expected=stock_mass if title=='stock' else stock_mass+header['delta_mass_g']-1000*rear['mass_removed_kg']
        if title=='everyday_guard': expected+=1000*guard['mass_kg']
        assert all(abs(q['robot_mass_g']-expected)<.002 for q in rolls)
    sources=[PROFILE,RAILS/'receiver-profile.json',RAILS/'plain-guard-profile.json',
             *[runtime/'python'/('xgoduck_'+name+'.onnx') for name in ('walk','getup','roulade')]]
    def mass_only(spec):
        stripped(spec)
        configure_rails(spec,dict(guard,collider_hulls_m=[]))
    def diagnostic(vx,yaw,**kwargs):
        kwargs['configure']=mass_only
        return rig['WalkSimulation'](vx,yaw,**kwargs)
    mass_only_trials={}
    for case in ('back','right'):
        row=rig['run_case'](case,[],{},simulation_factory=diagnostic)
        row['passed']=bool(rig['passes'](case,row,results['stock']['policy'][case]))
        assert row['mass_g']==results['everyday_guard']['policy'][case]['mass_g']
        mass_only_trials[case]=row
        print('guard mass with contact off',case,row['passed'],row['recovered_s'],flush=True)
    report=dict(head_delta_g=header['delta_mass_g'],receiver_delta_g=-1000*rear['mass_removed_kg'],
                everyday_cover_and_cartridges_g=1000*guard['mass_kg'],
                sha256={p.name:sha(p) for p in sources},trials=results,
                diagnostic_guard_mass_present_hulls_removed=mass_only_trials)
    (HERE/'off-results.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('mode',choices=['profile','trials'])
    p.add_argument('--runtime',type=Path,default=RUNTIME)
    args=p.parse_args()
    if args.mode=='profile':
        head=head_profile(args.runtime)
        PROFILE.write_text(json.dumps(head,indent=2)+'\n',encoding='utf-8')
        print(head['delta_mass_g'])
    else: run(args.runtime)
