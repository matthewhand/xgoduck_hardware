"""V5 derivative rigid jaws with external nut bosses and independent side tooth rows (mm)."""
import argparse
import json
from pathlib import Path
import runpy
import struct

import numpy as np
from manifold3d import CrossSection, Manifold, OpType

HERE = Path(__file__).resolve().parent
CAD = runpy.run_path(str(HERE.parent / 'dino-beak.py'))
FIT = runpy.run_path(str(HERE.parent / 'verify-fit.py'))
FRONT = CAD['FRONT']
FACE = 49.2
RAIL_SKIN = 2.4
SCREW_LENGTH = 6.
NUT_X = 46.55


def union(parts):
    return Manifold.batch_boolean(parts, OpType.Add)


def x_prism(profile, x0, x1):
    return profile.extrude(x1 - x0).transform(((0,0,1,x0),(1,0,0,0),(0,1,0,0)))


def x_cylinder(x0, x1, y, z, radius):
    return Manifold.cylinder(x1-x0,radius,radius,40).rotate((0,90,0)).translate((x0,y,z))


def yz_rect(y0,y1,z0,z1):
    return CrossSection([[(y0,z0),(y1,z0),(y1,z1),(y0,z1)]])


def nut(y,z,flat):
    r=flat/np.sqrt(3)
    return CrossSection([[(y+r*np.cos(t), z+r*np.sin(t)) for t in np.arange(6)*np.pi/3+np.pi/6]])


def block_at_mount(y,z):
    # A rectangular top-loading pocket is open during slicing and completely
    # roofed when printing finishes. Opposing Y flats restrain the nut.
    boss=x_cylinder(43.3,FACE,y,z,3.7)
    slot=yz_rect(y-2.11,y+2.11,z-2.34,z+2.51)
    boss-=x_prism(slot,NUT_X-.13,NUT_X+1.78)
    boss-=x_cylinder(45.1,FACE+.3,y,z,1.15)
    return boss.trim_by_plane((0,0,1),0)


def rail(side):
    upper=side=='upper'
    y0=FRONT[side]
    z=13.05 if upper else 3.0
    # Round-ended, flat-backed web. Two screw axes register its orientation.
    screw_y=[y0+4.6,y0+18.6]
    plates=[x_cylinder(FACE,FACE+RAIL_SKIN,y,z,2.95) for y in screw_y]
    web=Manifold.batch_hull(plates)
    ridge=CAD['rounded_span']((FACE+1.5,y0-.7,z),(FACE+1.5,y0+24.5,z),(1.0,1.15,1.65))
    # Raised rim recesses the full screw head while retaining the 2.4 mm web.
    parts=[web,ridge]+[x_cylinder(FACE+2.15,FACE+4.9,y,z,3.05) for y in screw_y]
    ys=[-.8,6.4,13.6,20.8] if upper else [2.2,9.4,16.6,23.8]
    root_z=z-1.05 if upper else z+1.05
    tip_z=7.05 if upper else 9.0
    for station in ys:
        y=y0+station
        root=[CAD['egg']((FACE+1.4,y+d,root_z),(1.07,.91,1.12)) for d in (-1.05,1.05)]
        tip=CAD['egg']((FACE+(4.5 if upper else 1.5),y,tip_z),(.94,.85,.9))
        parts.append(Manifold.batch_hull([*root,tip]))
    result=union(parts)
    hardware=[]
    for y in screw_y:
        result-=x_cylinder(FACE-.2,FACE+RAIL_SKIN+2,y,z,1.16)
        result-=x_cylinder(FACE+RAIL_SKIN,FACE+7,y,z,2.18)
        shank=x_cylinder(FACE+RAIL_SKIN-SCREW_LENGTH,FACE+RAIL_SKIN,y,z,1.)
        head=x_cylinder(FACE+RAIL_SKIN,FACE+RAIL_SKIN+2.,y,z,1.90)
        fastener=shank+head
        captive=x_prism(nut(y,z,4.0),NUT_X,NUT_X+1.6)-x_cylinder(NUT_X-.1,NUT_X+2,y,z,1.)
        hardware.extend([fastener,captive])
    return result, union([block_at_mount(y,z) for y in screw_y]), hardware


def pair(solid):
    return solid+solid.mirror((1,0,0))


def build():
    result={}
    for side,name in [('upper','HEAD_BOTTOM'),('lower','JAW')]:
        stock=CAD['read_stock'](name)
        strip,bosses,hardware=rail(side)
        mod=stock+pair(bosses)
        assert mod.status()==CAD['Error'].NoError
        # Upstream HEAD_BOTTOM has an enclosed void with its own inward-facing shell.
        assert sum(piece.volume()>0 for piece in mod.decompose())==1
        assert (stock-mod).volume()<1e-4
        CAD['check_solid'](strip)
        net=mod-stock
        metal=pair(union(hardware))
        result[side]=dict(stock=stock,mod=mod,bosses=net,strip=strip,rows=pair(strip),metal=metal,hardware=hardware)
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
    path.write_bytes(b'V5 external screw bosses, mm'.ljust(80,b' ')+struct.pack('<I',len(tris))+out.tobytes())
    exported=np.frombuffer(path.read_bytes(),dtype=CAD['DTYPE'],offset=84)['vertex']
    pts,ix=np.unique(exported.reshape(-1,3),axis=0,return_inverse=True)
    ix=ix.reshape(-1,3)
    edges=np.concatenate([ix[:,[0,1]],ix[:,[1,2]],ix[:,[2,0]]])
    assert np.all(np.unique(np.sort(edges,axis=1),return_counts=True,axis=0)[1]==2)
    assert abs(float(pts[:,2].min()))<.01


def export(parts,output):
    output.mkdir(parents=True,exist_ok=True)
    for side,r in parts.items():
        write_host(bed(r['mod']),output/(side+'-jaw-with-bosses.stl'))
        for sign,label in [(1,'plus-x'),(-1,'minus-x')]:
            item=r['strip'] if sign==1 else r['strip'].mirror((1,0,0))
            # Place the mating face on the bed. Each row stands on a broad flat web.
            item=item.rotate((0,-90*sign,0))
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
    checks={}
    def clearance(name,a,b,limit=10):
        d=limit if b.is_empty() else float(a.min_gap(b,limit))
        checks[name]=min(checks.get(name,limit),d)
        if d<.001:
            raise ValueError(f'{name} collision: {(a^b).volume():.4f} mm3')
    for side,r in parts.items():
        pad=top_pad if side=='upper' else bottom_pad
        clearance(side+' metal vs original rigid wall',r['metal'],r['stock'])
        clearance(side+' metal vs same-side pad',r['metal'],pad)
        clearance(side+' cheek strip vs original rigid wall',r['rows'],r['stock'])
        clearance(side+' bosses vs same-side pad',r['bosses'],pad)
        assert (r['metal']^r['mod']).volume()<.001
        assert (r['metal']^r['rows']).volume()<.001
        assert (r['mod']^r['rows']).volume()<.001
    fixed=union([head['bosses'],head['rows'],head['metal']])
    moving=union([jaw['bosses'],jaw['rows'],jaw['metal']])
    clearance('upper fittings vs face',fixed,face)
    clearance('upper fittings vs ToF housing',fixed,tof)
    def optical(solid):
        # Largest comparison FOV, using the same nominal forward-facing
        # site/registration as ../check-camera.py in upper print coordinates.
        m=solid.to_mesh()
        v=m.vert_properties[:,:3]
        lateral=np.abs(-v[:,0]-.00325 - .37724792)
        depth=- (v[:,1]-17.05394 + 76.575988)
        return float(np.min(lateral)-max(0,float(np.max(depth)))*(640/480)*np.tan(np.deg2rad(55/2)))
    view_margin=optical(head['metal'])
    # At the nearest new mount |x|>43. Extra clearance queries are capped at
    # 10 mm. All original geometry inside |x|<32 is beyond that cap.
    region=pair(Manifold.cube((43,220,150)).translate((32,-110,-50)))
    jaw_zone=jaw['stock']^region
    head_zone=head['stock']^region
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
    return {'sampled_jaw_deg':[0,30,.25],'minimum_clearances_mm_capped_at_10':checks,
            'metal_outside_55deg_vfov_horizontal_margin_mm':view_margin,
            'net_rigid_volume_cm3':{s:r['bosses'].volume()/1000 for s,r in parts.items()},
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
