"""V5 derivative jaws with concealed square nuts and independent removable cheek teeth."""
import argparse
import json
from pathlib import Path
import runpy
import struct

import numpy as np
from manifold3d import Manifold, OpType

HERE = Path(__file__).resolve().parent
CAD = runpy.run_path(str(HERE.parent / 'dino-beak.py'))
FIT = runpy.run_path(str(HERE.parent / 'verify-fit.py'))
FRONT = CAD['FRONT']
NUT_X = 42.10
SEAT_X = 47.05
SCREW_LENGTH = 6.
MOUNT = {'upper': (11.50, (31.70,36.60)), 'lower':(5.00,(7.,21.))}


def union(parts):
    return Manifold.batch_boolean(parts, OpType.Add)


def pair(solid):
    return solid + solid.mirror((1,0,0))


def box(c, size):
    return Manifold.cube(size,True).translate(c)


def x_cylinder(x0,x1,y,z,radius):
    return Manifold.cylinder(x1-x0,radius,radius,32).rotate((0,90,0)).translate((x0,y,z))


def sites(side):
    z, reaches=MOUNT[side]
    return [(FRONT[side]+r,z) for r in reaches]


def rail(side,stock):
    upper=side=='upper'
    y0=FRONT[side]
    site=sites(side)
    z_rib=13.0 if upper else 2.6
    # The removable strip carries the outside seating pads and all screw-head rims.
    pads=[x_cylinder(43.4,SEAT_X,y,z,1.75) for y,z in site]
    if upper:
        pads.append(x_cylinder(43.4,SEAT_X,y0+13.5,11.6,1.5))
    ys=([11.,17.6,24.2,30.8] if upper else [2.5,9.1,15.7,22.3])
    ridge=CAD['rounded_span']((46.,y0+ys[0]-1.0,z_rib),(46.,site[-1][0]+1.1,z_rib),
                              (1.12,1.4,1.50))
    pieces=[ridge,*pads]
    for y,z in site:
        mount=x_cylinder(44.6,SEAT_X+2.5,y,z,2.67)
        local=CAD['egg']((46.,y,z_rib),(1.06,1.23,1.25))
        pieces.append(Manifold.batch_hull([local,mount]))
    if upper:
        pieces.append(CAD['rounded_span']((46.,y0+13.5,11.6),(46.,y0+13.5,z_rib),(1.1,1.45,.95)))
    for reach in ys:
        y=y0+reach
        root_z=z_rib+(-.75 if upper else .75)
        roots=[CAD['egg']((46.,y+d,root_z),(1.00,.95,.96)) for d in (-.85,.85)]
        tip=CAD['egg'](((49.0 if upper else 45.9),y,(7.95 if upper else 7.9)),(.80,.80,.78))
        pieces.append(Manifold.batch_hull([*roots,tip]))
    # Right and left strips share the same seating profile. Take the envelope
    # of both mesh sides to clear the small asymmetry in the official STL.
    raw=union(pieces)-stock-stock.mirror((1,0,0))
    cutters=[];hardware=[];closed_pockets=[]
    for y,z in site:
        pocket=box((NUT_X,y,z),(1.94,4.22,4.23))
        channel=x_cylinder(40.80,45.5,y,z,1.15)
        cutters.extend([pocket,channel]);closed_pockets.append(pocket)
        raw-=x_cylinder(41.,SEAT_X+.2,y,z,1.16)
        raw-=x_cylinder(SEAT_X,52.,y,z,2.08)
        screw=x_cylinder(SEAT_X-SCREW_LENGTH,SEAT_X,y,z,1.0)+x_cylinder(SEAT_X,SEAT_X+2.,y,z,1.9)
        nut=box((NUT_X,y,z),(1.6,4.,4.))-x_cylinder(40.,45.,y,z,1.)
        hardware.extend([screw,nut])
    # Put the outer screw rings and the upper tips on a common print datum.
    raw=raw.trim_by_plane((-1,0,0),-(SEAT_X+2.5))
    return raw,pair(union(cutters)),pair(union(closed_pockets)),hardware


def build():
    result={}
    for side,name in [('upper','HEAD_BOTTOM'),('lower','JAW')]:
        stock=CAD['read_stock'](name)
        strip,cutters,pockets,hardware=rail(side,stock)
        mod=stock-cutters
        assert mod.status()==CAD['Error'].NoError
        assert sum(piece.volume()>0 for piece in mod.decompose())==1
        assert (mod-stock).volume()<1e-5
        assert (pockets-stock).volume()<.0001, side+' nut cavity breaks stock outline'
        CAD['check_solid'](strip)
        result[side]=dict(stock=stock,mod=mod,pockets=pockets,strip=strip,rows=pair(strip),
                          metal=pair(union(hardware)),hardware=hardware)
    return result



def bed(solid):
    return solid.translate((0,0,-solid.bounding_box()[2]))


def write_host(part,path):
    # Preserve the upstream enclosed void instead of discarding its negative shell.
    mesh=part.to_mesh()
    tris=mesh.vert_properties[mesh.tri_verts,:3].copy()
    normals=np.cross(tris[:,1]-tris[:,0],tris[:,2]-tris[:,0])
    lengths=np.linalg.norm(normals,axis=1)
    assert np.all(lengths>0)
    out=np.zeros(len(tris),dtype=CAD['DTYPE'])
    out['normal']=normals / lengths[:,None]
    out['vertex']=tris
    path.write_bytes(b'V5 flush screw mounts, mm'.ljust(80,b' ')+struct.pack('<I',len(tris))+out.tobytes())
    exported=np.frombuffer(path.read_bytes(),dtype=CAD['DTYPE'],offset=84)['vertex']
    pts,ix=np.unique(exported.reshape(-1,3),axis=0,return_inverse=True)
    ix=ix.reshape(-1,3)
    edges=np.concatenate([ix[:,[0,1]],ix[:,[1,2]],ix[:,[2,0]]])
    assert np.all(np.unique(np.sort(edges,axis=1),return_counts=True,axis=0)[1]==2)
    assert abs(float(pts[:,2].min()))<.01


def export(parts,output):
    output.mkdir(parents=True,exist_ok=True)
    for side,r in parts.items():
        write_host(bed(r['mod']),output/(side+'-jaw-flush-receivers.stl'))
        for sign,label in [(1,'plus-x'),(-1,'minus-x')]:
            item=r['strip'] if sign==1 else r['strip'].mirror((1,0,0))
            # Print from the coplanar outside rims. Support the raised web.
            item=item.rotate((0,90*sign,0))
            CAD['write_stl'](bed(item),output/(side+'-'+label+'-four-teeth.stl'))
    for path in sorted(output.glob('*-four-teeth.stl')):
        FIT['check_export'](path)


def verify(parts):
    head,jaw=parts['upper'],parts['lower']
    jaw_pose=FIT['jaw_pose']
    top_pad=CAD['read_stock']('MOUTH_UP_TPU').rotate((180,0,0)).translate((0,-43.840,11))
    bottom_pad=CAD['read_stock']('MOUTH_BOTTOM_TPU').translate((0,-15.54,2.8))
    face=CAD['read_stock']('FACE').transform(((0,-1,0,0),(0,0,1,-54.848),(-1,0,0,33.258)))
    tof=CAD['read_stock']('FACE_TOF').transform(((1,0,0,24.693),(0,0,-1,-48.048),(0,1,0,30.627)))
    checks={}; wall={}
    def clearance(name,a,b,limit=2.):
        d=limit if b.is_empty() else float(a.min_gap(b,limit))
        checks[name]=min(checks.get(name,limit),d)
        if d<.0005:
            raise ValueError(f'{name} contact near {angle} degrees')
    angle='static'
    for side,r in parts.items():
        pad=top_pad if side=='upper' else bottom_pad
        wall[side]=[]
        for y,z in sites(side):
            # The padded entire nut recess and the bore's blind end must
            # remain inside original material on BOTH side walls.
            skin=pair(box((NUT_X,y,z),(1.94+.5,4.22+.5,4.23+.5)))
            end=pair(x_cylinder(40.55,40.8,y,z,1.15))
            assert (skin-r['stock']).volume()<.0001
            assert (end-r['stock']).volume()<.0001
            hits=r['stock'].ray_cast((48,y,z),(40.80,y,z))
            assert len(hits)==1
            wall[side].append(round(hits[0].position[0]-40.8,3))
        clearance(side+' metal vs same-side pad',r['metal'],pad)
        clearance(side+' strip vs same-side pad',r['rows'],pad)
        assert (r['metal']^r['mod']).volume()<.001
        assert (r['metal']^r['rows']).volume()<.001
        assert (r['mod']^r['rows']).volume()<.001
    fixed=head['rows']+head['metal']
    moving=jaw['rows']+jaw['metal']
    clearance('upper fittings vs face',fixed,face)
    clearance('upper fittings vs ToF housing',fixed,tof)
    def optical(solid):
        # Same nominal forward-facing site and registration as ../check-camera.py.
        m=solid.to_mesh()
        v=m.vert_properties[:,:3]
        lateral=np.abs(-v[:,0]-.00325 - .37724792)
        depth=- (v[:,1]-17.05394 + 76.575988)
        return float(np.min(lateral)-max(0,float(np.max(depth)))*(640/480)*np.tan(np.deg2rad(55/2)))
    view_margin=optical(head['metal'])
    # New metal reaches x=41.05 mm. The 34 mm cut leaves over 7 mm
    # separation from the crop plane for our 2 mm capped queries.
    region=pair(Manifold.cube((46,220,150)).translate((34,-110,-50)))
    jaw_zone=jaw['mod']^region
    head_zone=head['mod']^region
    bottom_zone=bottom_pad^region
    top_zone=top_pad^region
    face_zone=face^region
    tof_zone=tof^region
    for angle in np.arange(121)*.25:
        m=jaw_pose(moving,angle)
        clearance('across jaws',fixed,m)
        clearance('upper fittings vs lower rigid jaw',fixed,jaw_pose(jaw_zone,angle))
        clearance('upper fittings vs lower pad',fixed,jaw_pose(bottom_zone,angle))
        clearance('lower fittings vs upper rigid jaw',m,head_zone)
        clearance('lower fittings vs upper pad',m,top_zone)
        clearance('lower fittings vs face',m,face_zone)
        clearance('lower fittings vs ToF housing',m,tof_zone)
        view_margin=min(view_margin,optical(jaw_pose(jaw['metal'],angle)))
    assert view_margin>0
    return {'sampled_jaw_deg':[0,30,.25],'minimum_clearances_mm_capped_at_2':checks,
            'metal_outside_55deg_vfov_horizontal_margin_mm':view_margin,
            'exterior_added_volume_mm3':{s:abs(round((r['mod']-r['stock']).volume(),6)) for s,r in parts.items()},
            'material_removed_cm3':{s:(r['stock']-r['mod']).volume()/1000 for s,r in parts.items()},
            'blind_bore_depth_on_axis_mm':wall,'pocket_stock_clearance_verified_mm':.25,
            'nut_roof_z_in_print_mm':{s:MOUNT[s][0]+4.23/2 for s in parts},
            'volume_per_four_tooth_strip_cm3':{s:r['strip'].volume()/1000 for s,r in parts.items()}}



if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=HERE/'print')
    args=parser.parse_args()
    parts=build()
    report=verify(parts)
    export(parts,args.output)
    (HERE/'fit.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf8')
    print(json.dumps(report,indent=2))
