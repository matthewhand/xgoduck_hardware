"""Generate an unpowered V1.1 charging-backpack fit prototype from Luwu's enclosure STL.

Assembly uses the source STL coordinates in mm. The pack enters from the head
(+Y) and lowers toward the feet (-Y). Smaller Z is farther from the electronics.
"""
import argparse
import hashlib
from pathlib import Path
import struct

import numpy as np
from manifold3d import CrossSection, Error, JoinType, Manifold, Mesh, OpType

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / 'structure/V5_AUDIO_VIDEO_/V5_AUDIO_VIDEO_ENCLOSURE_FOR_V1P1.stl'
SOURCE_SHA = '7cb6660912e891fe694a0bd5c0eb2060d132cec6e589849652eab39cfc56e92f'
LEFT, RIGHT = -36.3, 36.3
FACE = -3.2


def block(x0, x1, y0, y1, z0, z1):
    return Manifold.cube((x1-x0, y1-y0, z1-z0)).translate((x0,y0,z0))


def along_y(section, x, y0, y1):
    # Profile coordinates are lateral position and outward depth.
    return section.extrude(y1-y0).rotate((-90,0,0)).translate((x,y0,2.6))


def stock_enclosure():
    if hashlib.sha256(SOURCE.read_bytes()).hexdigest() != SOURCE_SHA:
        raise ValueError('Expected the pinned V5 V1.1 enclosure')
    dtype = np.dtype([('normal','<f4',3),('v','<f4',(3,3)),('attribute','<u2')])
    triangles = np.fromfile(SOURCE, dtype=dtype, offset=84)['v']
    points, indices = np.unique(triangles.reshape(-1,3), axis=0, return_inverse=True)
    return Manifold(Mesh(points, indices.reshape(-1,3).astype(np.uint32)))


def build(stock, clearance):
    profile = CrossSection([[(-2.8,-.2),(2.8,-.2),(4,2.6),(-4,2.6)]])
    groove = profile.offset(clearance,JoinType.Miter)
    fixed = stock
    # The rail sockets run outside the raised headward edge of the stock STL.
    outline = [(-31,-32),(31,-32),(34,-29),(34,22),(38.8,26),(38.8,33),
               (36,35),(-36,35),(-38.8,33),(-38.8,26),(-31,21)]
    cover = CrossSection([outline]).extrude(2.2).translate((0,0,FACE))
    # Keep the stock upper viewing/access aperture and lower cable exit open.
    cover -= block(-1,22.35,-25.2,3.8,-6,5)
    cover -= block(-1,41,8,22.2,-6,5)
    for x,y in [(-8,-31.9),(20,-31.9),(-23.9,18.9)]:
        cover -= Manifold.cylinder(8,4.1,4.1,32).translate((x,y,-6))
    for y in [-21,-15,-9,-3,3,9]:
        cover -= block(-18,-5,y,y+2,-6,5)

    for x in [LEFT, RIGHT]:
        root = block(x-2.8,x+2.8,-27,1,2.55,7.25)
        saddle = block(min(x,np.sign(x)*27.2),max(x,np.sign(x)*27.2),-27,1,4.4,7.25)
        stop = block(x-5.5,x+5.5,-29,-27,0,7.25)
        rail = along_y(profile,x,-27,1) + root + saddle + stop
        if x == LEFT:
            rail -= block(x-2.5,x+2.5,-19,-13.7,-.1,1.2)
        if (rail ^ stock).volume() < 35:
            raise ValueError('Insufficient union between rail root and stock enclosure')
        fixed += rail
        cover += block(min(x,np.sign(x)*29),max(x,np.sign(x)*29),-27,1.2,FACE,FACE+2.2)
        cover += block(x-5.7,x+5.7,-27,1.2,FACE,2.85)
        # Cut both ends of each groove. Its lower face bears on the fixed stop.
        cover -= along_y(groove,x,-28,2)
        cover -= block(x-2.8-clearance,x+2.8+clearance,-28,2,2.60,4)

    # A U-cut tongue carries an inward-facing ramp. Lift through the finger
    # relief to release the shoulder. No separate pin or retaining screw.
    for x in [LEFT-2.8,LEFT+1.9]:
        cover -= block(x,x+.9,-20.2,-1,-5,.1)
    cover -= block(LEFT-2.8,LEFT+2.8,-21.3,-18.8,-5,.1)
    # Thin the cantilever from the channel side, retaining a 1.35 mm flexure.
    cover -= block(LEFT-1.9,LEFT+1.9,-14.3,-2.5,FACE+1.35,.1)
    tooth = CrossSection([[(-18.6,-.65),(-14,-.65),(-14,.55),(-16.8,.55),(-18.6,-.10)]])
    tooth = tooth.extrude(3.5).transform(((0,0,1,LEFT-1.75),(1,0,0,0),(0,1,0,0)))
    # The 56 mm centre spacing comes from the fixed-pad chair experiment.
    # Fold both ends of each metal strip through its own pocket. One underside
    # groove per pole lets its lead leave the headward edge without crossing a rail.
    for x in [-28,28]:
        # Back the contact against the solid stock perimeter after 0.30 mm of
        # nominal deflection, leaving running clearance during installation.
        support_x = np.sign(x)*25
        cover += block(support_x-2,support_x+2,27.5,32.5,-1.1,-.30)
        cover -= block(x-10.25,x+10.25,28.55,31.45,FACE-.1,FACE+.45)
        for end in [x-9,x+9]:
            cover -= block(end-.6,end+.6,28.6,31.4,FACE,1)
        inner = x - np.sign(x)*9
        cover -= block(inner-1.25,inner+1.25,29,36,-1.8,0)
    return fixed, cover + tooth, cover, tooth


def checked(solid):
    if solid.status()!=Error.NoError or len(solid.decompose())!=1 or solid.volume()<=0:
        raise ValueError('Expected one connected closed solid')


def sweep(solid, distance):
    # Translate the solid plus the volume swept by every leading triangle.
    mesh=solid.to_mesh()
    tris=mesh.vert_properties[mesh.tri_verts,:3].astype(float)
    normals=np.cross(tris[:,1]-tris[:,0],tris[:,2]-tris[:,0])
    delta=np.array([0.,distance,0.])
    pieces=[solid]
    for tri in tris[normals @ delta > 1e-7]:
        pieces.append(Manifold.hull_points(np.vstack((tri,tri+delta))))
    return Manifold.batch_boolean(pieces,OpType.Add)


def write_stl(path,solid):
    checked(solid)
    mesh=solid.to_mesh()
    triangles=mesh.vert_properties[mesh.tri_verts,:3]
    cross=np.cross(triangles[:,1]-triangles[:,0], triangles[:,2]-triangles[:,0])
    lengths=np.linalg.norm(cross,axis=1)
    if np.any(lengths==0):
        raise ValueError('Degenerate exported triangle')
    with path.open('wb') as out:
        out.write(b'XGO Duck V5 V1.1 slide-on charging pack | mm'.ljust(80,b'\0'))
        out.write(struct.pack('<I',len(triangles)))
        for face,n in zip(triangles,cross/lengths[:,None]):
            out.write(struct.pack('<12fH',*n,*face.flat,0))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output',type=Path)
    parser.add_argument('--clearance',type=float,choices=(.15,.25,.35),default=.25)
    args=parser.parse_args()
    stock=stock_enclosure()
    fixed,pack,rigid,tooth=build(stock,args.clearance)
    for solid in [stock,fixed,pack,rigid]:
        checked(solid)
    seated=(pack ^ fixed).volume()
    # 0.85 mm outward travel clears the tooth for release. The flexible beam
    # moves away from the enclosure. This is a clearance test, not a force test.
    released = rigid + tooth.translate((0,0,-.85))
    slide=(sweep(released,76) ^ fixed).volume()
    forward=(released.translate((0,-.5,0)) ^ fixed).volume()
    back=(pack.translate((0,1.5,0)) ^ fixed).volume()
    print(f'Seated / rigid slide interference: {seated:.5f} / {slide:.5f} mm3')
    print(f'Stop / engaged withdrawal overlap: {forward:.3f} / {back:.3f} mm3')
    if max(seated,slide)>1e-4 or min(forward,back)<.1:
        raise ValueError('Invalid slide, stop or retention geometry')
    print(f'Added rail volume {(fixed.volume()-stock.volume())/1000:.2f} cm3; cover {pack.volume()/1000:.2f} cm3')
    if abs(fixed.bounding_box()[2])>1e-5 or abs(pack.bounding_box()[2]-FACE)>1e-5:
        raise ValueError('Printed parts must rest flat on their designed bed faces')
    args.output.mkdir(exist_ok=True,parents=True)
    write_stl(args.output/'duck-v5-v1p1-top-entry-rail-enclosure.stl',fixed)
    write_stl(args.output/'duck-v5-v1p1-top-entry-contact-backpack.stl',pack.translate((0,0,-FACE)))


if __name__=='__main__':
    main()
