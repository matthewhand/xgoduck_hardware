"""Four-bolt masquerade and two-bolt beard, in the registered V5 print frames (mm)."""
import json
import math
from pathlib import Path
import runpy

import numpy as np
from manifold3d import CrossSection, Manifold

HERE = Path(__file__).resolve().parent
BEAK = HERE.parent / 'v5-dino-beak'
cad = runpy.run_path(str(BEAK / 'dino-beak.py'))
jaw_pose = runpy.run_path(str(BEAK / 'verify-fit.py'))['jaw_pose']
FACE_POINTS = [(s*x, z) for s in (-1, 1) for x, z in [(17, 44), (28, 40.5)]]
CHIN_POINTS = [(-11, 23), (11, 23)]
FACE_Y = -54.848
BROW_EXTENSION = 5.65


def cyl(x, y, z, h, r, sides=48):
    return Manifold.cylinder(h, r, r, sides).translate((x,y,z))


def slab(points, bottom, thickness):
    area2 = sum(x*v-u*y for (x,y),(u,v) in zip(points,points[1:]+points[:1]))
    return CrossSection([points if area2>0 else points[::-1]]).extrude(thickness).translate((0,0,bottom))


def join(parts):
    return sum(parts[1:], parts[0])


def boss(x,y):
    # The square nut slides in radially from the positive local Y edge.
    # Enclosing material takes axial load. The screw blocks sideways escape.
    solid = cyl(x,y,-.40,5.70,4.2)
    pocket = Manifold.cube((4.35,7,1.90)).translate((x-2.175,y-2.175,1.25))
    return solid - pocket - cyl(x,y,.55,5.1,1.2)


def plate_holes(part,points):
    for x,y in points:
        part -= cyl(x,y,5,4.2,1.20)
        part -= cyl(x,y,6.90,4,2.15)
    return part


def mask():
    # A sweeping wing and a pierced lower rim leave the camera and ranging apertures open.
    wing = [(0,40.8),(4,40.0),(9,39.5),(14,38.8),(10.6,34),(12,29),
            (17,24.8),(23,22.8),(30.5,24.5),(35,28),(40,33),(43,37),
            (39,36),(45,43),(39,41),(42,47),(36,44),(34,46),
            (29,47.8),(21,48.6),(14,50.4),(8,51),(4,52),(0,53.5)]
    eye = [(13.8,34.4),(16.3,37.5),(22.5,38.1),(29.5,37.5),(36,35.1),
           (32.2,30.5),(28.4,27.6),(23.1,26.2),(17.6,28.1),(14.5,30.7)]
    half = slab(wing,5.3,2.9) - slab(eye,5,4)
    # Three incised feather-shaped channels at the temple.
    for x,y in [(32,40),(34.2,38.7),(36,37.4)]:
        half -= slab([(x-1,y-.35),(x+3.9,y+2),(x+.15,y+.65)],7.55,1.1)
    rim = half + half.mirror((1,0,0))
    rim -= cyl(-.32,31.6,4.9,4,16.65,120)
    for x,y in FACE_POINTS:
        rim += cyl(x,y,5.3,2.9,3.65)
    rim = plate_holes(rim, FACE_POINTS).translate((0,0,BROW_EXTENSION))
    # Flare the openings out to the projected front of the mask.
    for x in (-19.62,19.62):
        rim -= Manifold.cylinder(30, .55, 30.55, 96).translate((x,32.116,4.674))
    return rim


def beard():
    # First two coordinates are the underside of the jaw. Local Z points down.
    ends=[cyl(x,y,5.3,2.9,4.25) for x,y in CHIN_POINTS]
    bar=Manifold.batch_hull(ends)
    # A thin scalloped fan turns down from the chin. Build its profile in X/Z.
    outline=[(-15,6.6),(-8,7.1),(0,7.35),(8,7.1),(15,6.6),(14.2,11),
             (12.2,14.2),(8.7,16.5),(9.3,13.5),(4.1,19),(1.2,20.5),
             (0,23),(-2,21.4),(-6.4,17.3),(-7.2,15.1),(-7,18.2),
             (-11,15.1),(-14.2,11)]
    fan=slab(outline,0,2.2).rotate((90,0,0)).translate((0,27,0))
    return plate_holes(bar + fan, CHIN_POINTS)


def front(p):
    return p.rotate((90,0,0)).translate((0,FACE_Y,0))


def chin(p):
    return p.rotate((180,0,0))


def to_face_print(p):
    # Local face-plate XYZ -> stock FACE print XYZ.
    return p.transform(((0,-1,0,33.258),(-1,0,0,0),(0,0,-1,0)))


def build():
    face,jaw=cad['read_stock']('FACE'),cad['read_stock']('JAW')
    brows=join([boss(x,y).translate((0,0,BROW_EXTENSION)) +
                cyl(x,y,-.4,BROW_EXTENSION+.2,3.65) for x,y in FACE_POINTS])
    chin_boss=join([boss(*q) for q in CHIN_POINTS])
    return (face+to_face_print(brows), jaw+chin(chin_boss), mask(), beard(), brows, chin_boss)


def seat(p):
    return p.translate((0,0,-p.bounding_box()[2]))


def metal(points):
    return join([cyl(x,y,.90,6,1)+cyl(x,y,6.90,1.05,1.9) for x,y in points])


def verify(parts):
    face,jaw,plate,fringe,brows,under=parts
    for p in parts[:4]:
        cad['check_solid'](p)
    active = front(plate+brows)
    fixed = cad['read_stock']('HEAD_BOTTOM')
    tof=cad['read_stock']('FACE_TOF').transform(((1,0,0,24.693),(0,0,-1,-48.048),(0,1,0,30.627)))
    fixed_pad=cad['read_stock']('MOUTH_UP_TPU').rotate((180,0,0)).translate((0,-43.840,11))
    moving=chin(fringe+under)
    fixture=active+fixed+fixed_pad+tof
    gap=10
    for a in np.arange(0,30.01,.25):
        gap=min(gap, jaw_pose(moving,a).min_gap(fixture,10),
                jaw_pose(jaw,a).min_gap(active,10))
    assert gap>.2, gap
    for stock,convert,fasteners,points in [(cad['read_stock']('FACE'),to_face_print,brows,FACE_POINTS),
                                            (cad['read_stock']('JAW'),chin,under,CHIN_POINTS)]:
        offset=(0,0,BROW_EXTENSION if points==FACE_POINTS else 0)
        assert (convert(metal(points).translate(offset)) ^ stock).volume()<.0001
        assert (metal(points).translate(offset) ^ fasteners).volume()<.0001
        for x,y in points:
            # An intact annulus beneath the head must survive the optical cutouts.
            bearing=cyl(x,y,6.65,.24,1.94)-cyl(x,y,6.64,.30,1.22)
            bearing=bearing.translate(offset)
            flange=plate if points==FACE_POINTS else fringe
            assert (bearing ^ flange).volume() >= .999*bearing.volume()
            assert (convert(cyl(x,y,-.4,.4,3.5)) ^ stock).volume()>13
            nut=Manifold.cube((4,4,1.6)).translate((x-2,y-2,1.40))
            nut-=cyl(x,y,1,3,1.05)
            assert (nut.translate(offset) ^ fasteners).volume()<.0001
    # Broad geometric sight cones looking forward from the two MJCF sites.
    optical={}
    for name,x in [('camera',-.3805),('tof',19.6195)]:
        cone=Manifold.cylinder(32,.1,32*math.tan(math.radians(45)),64).rotate((90,0,0)).translate((x,-59.52205,32.116))
        optical[name]=(active^cone).volume()
        assert optical[name]<.0001, optical
    preview=runpy.run_path(str(BEAK/"export-preview.py"))
    from manifold3d import Mesh
    # The stock exterior mesh includes the FACE mating surface and is an open
    # triangle soup. Check its skin directly beyond the intended boss roots.
    exterior=active ^ Manifold.cube((150,40,100)).translate((-75,FACE_Y-40.08,-10))
    em=exterior.to_mesh()
    triangles=em.vert_properties[em.tri_verts,:3]
    emin,emax=triangles.min(1),triangles.max(1)
    lo,hi=np.array(exterior.bounding_box()).reshape(2,3)
    for name,_,_,v,tri in preview["stock_head"](HERE.parents[2]/"xgoduck_runtime_arduino"):
        if name=="head-shell":
            vertices=(v-preview["OFFSET"]["upper"])@preview["PRINT_TO_HEAD"]
            shell=vertices[tri]
            for a in shell[((shell.min(1)<hi)&(shell.max(1)>lo)).all(1)]:
                if not ((a.min(0)<emax)&(a.max(0)>emin)).all(1).any():
                    continue
                normal=np.cross(a[1]-a[0],a[2]-a[0])
                normal*=.02/max(np.linalg.norm(normal),1e-9)
                skin=Manifold.hull_points(np.concatenate([a+normal,a-normal]))
                assert (skin^exterior).volume()<1e-6, a.mean(0)
        if name=="head-hardware":
            vertices=(v-preview["OFFSET"]["upper"])@preview["PRINT_TO_HEAD"]
            hardware=Manifold(Mesh(vertices.astype("float32"),tri.astype("uint32")))
            hardware_gap=hardware.min_gap(active,10)
            assert hardware_gap>.2, hardware_gap
    return dict(registered_head_hardware_gap_mm=round(hardware_gap,3),jaw_sweep_deg=[0,30,.25],minimum_clearance_mm_capped_at_10=round(gap,3),
                assumed_optical_half_angle_deg=45,cone_intersection_mm3=optical,
                accessory_plastic_g_at_1p2={k:round(v.volume()*.0012,2) for k,v in [('mask',plate),('beard',fringe)]})


def main():
    parts=build()
    report=verify(parts)
    face,jaw,plate,fringe,brows,under=parts
    # Bed the flat face of the fan. The screw recesses face sideways.
    outputs={'face-four-nut-bosses':face, 'jaw-two-chin-nut-bosses':jaw,
             'venetian-mask':plate,'scalloped-beard':fringe.rotate((90,0,0)),
             'nut-coupon':boss(0,0), 'bolt-coupon':plate_holes(cyl(0,0,5.3,2.9,4.25),[(0,0)])}
    teeth_path=BEAK/'screw-teeth'/'screw-teeth.py'
    side=runpy.run_path(str(teeth_path))['build']()['lower']['mod']
    outputs['jaw-side-teeth-and-chin']=side+chin(under)
    out=HERE/'print'
    out.mkdir(exist_ok=True)
    for name,p in outputs.items():
        path=out/(name+'.stl')
        cad['write_stl'](seat(p),path)
        runpy.run_path(str(BEAK/'verify-fit.py'))['check_export'](path)
    (HERE/'fit.json').write_text(json.dumps(report,indent=2)+'\n')
    print(report)


if __name__=='__main__':
    main()
