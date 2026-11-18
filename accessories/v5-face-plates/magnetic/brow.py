"""Blind steel targets in the V5 FACE and four print-captive magnets in a removable mask."""
import json
from pathlib import Path
import runpy

import numpy as np
from manifold3d import Error, Manifold, Mesh

HERE=Path(__file__).resolve().parent
BASE=runpy.run_path(str(HERE.parent/'face-plates.py'))
CAD=BASE['cad']
join,cyl,front,to_print=(BASE[n] for n in ('join','cyl','front','to_face_print'))
POINTS=[(s*x,y) for s in (-1,1) for x,y in [(17,44),(30.1,39.6)]]
MAGNET_D=4.0
STEEL_D=4.0


def taper(z0,z1,r0,r1):
    return Manifold.cylinder(z1-z0,r0,r1,64).translate((0,0,z0))


def target_void():
    # Rear is local -Z. Stock front skin z=-0.70..0 is completely untouched.
    return join([taper(-2.26,-1.94,2.36,2.36),taper(-1.94,-1.73,2.36,2.62),
                 taper(-1.73,-1.52,2.62,2.62),taper(-1.52,-1.32,2.62,2.18),
                 taper(-1.32,-.70,2.18,2.18)])


def keeper():
    # Four independent compliant sections of rim snap behind the rear shoulder.
    k=join([taper(-2.16,-1.97,2.25,2.25),taper(-1.97,-1.72,2.25,2.54),
            taper(-1.72,-1.59,2.54,2.54),taper(-1.59,-1.37,2.54,2.08)])
    for angle in [0,90,180,270]:
        k-=Manifold.cube((1.15,.52,1.3)).translate((1.52,-.26,-2.4)).rotate((0,0,angle))
    for x in [-.82,.82]:
        k-=cyl(x,0,-2.3,1.1,.47)
    return k


def repeat(solid):
    return join([solid.translate((x,y,0)) for x,y in POINTS])


def mask_local():
    rim=BASE['mask']()
    extension=BASE['BROW_EXTENSION']
    for old,new in zip(BASE['FACE_POINTS'],POINTS):
        # Fill the screw pockets, then support from the new blind target centres.
        rim+=cyl(*old,5.30+extension,2.90,2.20)
        foot=cyl(*new,0,4.2,3.05)
        post=Manifold.batch_hull([cyl(*new,4.0,.25,3.05),cyl(*old,10.80,.43,3.10)])
        rim+=foot+post
        rim-=cyl(*new,.68,2.28,2.14)
    return rim


def magnetic_face():
    return CAD['read_stock']('FACE')-to_print(repeat(target_void()))


def target_discs():
    return repeat(cyl(0,0,-1.23,.50,STEEL_D/2))


def mask_magnets():
    return repeat(cyl(0,0,.76,2,MAGNET_D/2))


def rear_keepers():
    return repeat(keeper())


def check_mask(solid,pockets=4):
    shells=solid.decompose()
    assert solid.status()==Error.NoError and len(shells)==pockets+1
    assert sum(s.volume()>0 for s in shells)==1
    assert all(31<abs(s.volume())<34 for s in shells if s.volume()<0)


def write_mask(solid,path,pockets=4):
    solid=solid.simplify(.004)
    check_mask(solid,pockets)
    mesh=solid.to_mesh()
    triangles=mesh.vert_properties[mesh.tri_verts,:3]
    normals=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])
    lengths=np.linalg.norm(normals,axis=1)
    assert np.all(lengths>0)
    data=np.zeros(len(triangles),dtype=CAD['DTYPE'])
    data['normal']=normals/lengths[:,None]
    data['vertex']=triangles
    path.write_bytes(b'V5 closed magnet cavities mm'.ljust(80,b' ')+len(data).to_bytes(4,'little')+data.tobytes())
    loaded=np.frombuffer(path.read_bytes()[84:],dtype=CAD['DTYPE'])['vertex']
    points,indices=np.unique(loaded.reshape(-1,3),axis=0,return_inverse=True)
    faces=indices.reshape(-1,3).astype('uint32')
    edges=np.sort(np.concatenate([faces[:,[0,1]],faces[:,[1,2]],faces[:,[2,0]]]),axis=1)
    assert np.all(np.unique(edges,axis=0,return_counts=True)[1]==2)
    check_mask(Manifold(Mesh(points,faces)),pockets)


def check():
    stock=CAD['read_stock']('FACE')
    face=magnetic_face()
    plate=mask_local()
    keeps,steel,magnets=rear_keepers(),target_discs(),mask_magnets()
    for p in (face,keeper()):
        CAD['check_solid'](p)
    check_mask(plate)
    # Full outside skin, an extra 1 mm around each pocket and the full insertion
    # path with the FACE removed. No stock material at the original rear ribs is removed.
    for x,y in POINTS:
        skin=to_print(cyl(x,y,-.70,.70,2.65))
        assert (skin-face).volume()<.0001
        outer=to_print(cyl(x,y,-2.195,2.19,3.62))
        assert (outer-stock).volume()<.0001,(x,y,(outer-stock).volume())
        path=to_print(cyl(x,y,-18,15.79,2.55))
        assert (path^stock).volume()<.0001,(x,y,(path^stock).volume())
    assert (face^to_print(steel+keeps)).volume()<.0001
    assert (steel^keeps).volume()<.0001
    assert (plate^magnets).volume()<.0001
    assert (plate ^ repeat(cyl(0,0,0,.67,2.14))).volume()>4*.99*cyl(0,0,0,.67,2.14).volume()
    assert (plate ^ repeat(cyl(0,0,2.97,.60,2.14))).volume()>4*.99*cyl(0,0,2.97,.60,2.14).volume()
    assert (to_print(plate)^face).volume()<.0001
    blocked=(to_print(keeps.translate((0,0,-.36)))^face).volume()
    assert blocked>.10,blocked
    active=front(plate)
    fixed=CAD['read_stock']('HEAD_BOTTOM')
    tof=CAD['read_stock']('FACE_TOF').transform(((1,0,0,24.693),(0,0,-1,-48.048),(0,1,0,30.627)))
    pad=CAD['read_stock']('MOUTH_UP_TPU').rotate((180,0,0)).translate((0,-43.840,11))
    for s in (fixed,tof,pad):
        assert (active^s).volume()<.0001
    optics={}
    for name,x in [('camera',-.3805),('tof',19.6195)]:
        cone=Manifold.cylinder(32,.1,32.10,96).rotate((90,0,0)).translate((x,-59.52205,32.116))
        optics[name]=round((active^cone).volume(),6)
        assert optics[name]<.0001,optics
    preview=runpy.run_path(str(BASE['BEAK']/'export-preview.py'))
    runtime=HERE.parents[3]/'xgoduck_runtime_arduino'
    outer=active ^ Manifold.cube((150,40,100)).translate((-75,BASE['FACE_Y']-40.08,-10))
    mesh=outer.to_mesh()
    tris=mesh.vert_properties[mesh.tri_verts,:3]
    emin,emax=tris.min(1),tris.max(1)
    lo,hi=np.array(outer.bounding_box()).reshape(2,3)
    for name,_,_,vertices,faces in preview['stock_head'](runtime):
        v=(vertices-preview['OFFSET']['upper'])@preview['PRINT_TO_HEAD']
        if name=='head-hardware':
            hardware=Manifold(Mesh(v.astype('float32'),faces.astype('uint32')))
            housing_gap=hardware.min_gap(active,10)
            assert housing_gap>.2,housing_gap
        if name=='head-shell':
            shell=v[faces]
            for tri in shell[((shell.min(1)<hi)&(shell.max(1)>lo)).all(1)]:
                if not ((tri.min(0)<emax)&(tri.max(0)>emin)).all(1).any(): continue
                n=np.cross(tri[1]-tri[0],tri[2]-tri[0])
                n*=.02/max(np.linalg.norm(n),1e-9)
                assert (Manifold.hull_points(np.vstack([tri+n,tri-n]))^outer).volume()<1e-6,tri.mean(0)
    jaw=CAD['read_stock']('JAW')
    gap=min(BASE['jaw_pose'](jaw,a).min_gap(active,15) for a in np.arange(0,30.01,.25))
    assert gap>.2
    return dict(points_local_mm=POINTS,stock_front_skin_mm=.70,magnet_end_skin_mm=.68,
                seated_target_steel_to_magnet_distance_mm=1.49,keeper_shoulder_overlap_on_pull_mm3=round(blocked,3),
                rear_entry_clear_radius_mm=2.55,pocket_radial_stock_wall_at_least_mm=1.0,
                sensor_half_angle_deg=45,optical_overlap_mm3=optics,head_hardware_clearance_mm=round(housing_gap,3),
                jaw_range_deg=[0,30,.25],min_jaw_gap_mm_capped_at_15=round(gap,3),
                face_net_removed_cm3=round((stock.volume()-face.volume())/1000,4),
                mask_plastic_g_at_1p2=round(plate.volume()*.00120,3))


def main():
    report=check()
    out=HERE/'print'
    out.mkdir(exist_ok=True,parents=True)
    # Print from the decorated mask front toward the stand-offs' flat face pads.
    mask=BASE['seat'](mask_local().rotate((180,0,0)))
    report['mask_pause_reference_mm_from_bed']=round(mask.bounding_box()[5]-.68,3)
    coupon=(cyl(0,0,-2.2,2.2,5)-target_void()).rotate((180,0,0))
    foot=cyl(0,0,0,4.6,3.05)+cyl(0,0,3.45,1.15,5.4)-cyl(0,0,.68,2.28,2.14)
    foot=BASE['seat'](foot.rotate((180,0,0)))
    report['foot_coupon_pause_reference_mm_from_bed']=round(foot.bounding_box()[5]-.68,3)
    for name,solid in [('face-blind-steel-pockets',magnetic_face()),('magnetic-mask',mask),
                       ('inside-target-keeper',keeper()),('inside-target-coupon',coupon),
                       ('magnetic-foot-coupon',foot)]:
        path=out/(name+'.stl')
        if name in ('magnetic-mask','magnetic-foot-coupon'):
            write_mask(solid,path,4 if name=='magnetic-mask' else 1)
        else:
            CAD['write_stl'](BASE['seat'](solid),path)
            runpy.run_path(str(BASE['BEAK']/'verify-fit.py'))['check_export'](path)
    (HERE/'fit.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(report,flush=True)


if __name__=='__main__':
    main()
