"""V5 removable TPU front-lip cuffs. Assembly coordinates are the upstream part frames in mm."""
import argparse
import hashlib
from pathlib import Path
import struct
import subprocess

from manifold3d import CrossSection, Error, JoinType, Manifold, Mesh, OpType
import numpy as np


REVISION = '4496fad52041db1cad36ded6cc49d942e037b4af'
ROOT = Path(__file__).resolve().parents[2]
HASHES = {
    'HEAD_BOTTOM': '69789f906e1298d3d3af367eea0c65d7e844c329648c181acb6f01176d462cb3',
    'JAW': 'a5f6f4df0f6c5147a09840749db5259bbb3225bc34d409426e37548e7e913344',
    'MOUTH_UP_TPU': '119ffdfbb2e217d126c8515c37063f0a15cdb1c04fe5afef8f853dfb1896e8ee',
    'MOUTH_BOTTOM_TPU': 'fd141f4d94f74fe6db0ca2441cd10105c69aa93672a8f9ffc0bc36cb23029323',
    'FACE': '1897a5b46ae9579886001b6aecbdb28d783eaf6bdb124e266afa969e23024b51',
    'FACE_TOF': '9296e5bc6b5cde2b85ca2c5e99c8b0e54143882da669c65bd1f0c2d38de5736d',
}
DTYPE = np.dtype([('normal','<f4',3), ('vertex','<f4',(3,3)), ('attribute','<u2')])
FRONT = {'upper': -59.445, 'lower': -33.848}


def read_stock(name):
    rel = f'structure/V5_AUDIO_VIDEO_/V5_AUDIO_VIDEO_{name}.stl'
    path = ROOT / rel
    data = path.read_bytes() if path.exists() else subprocess.check_output(
        ['git','-C',str(ROOT),'show',f'{REVISION}:{rel}'])
    if hashlib.sha256(data).hexdigest() != HASHES[name]:
        raise ValueError(f'Source mesh differs from the registered V5 part: {name}')
    tris = np.frombuffer(data, dtype=DTYPE, offset=84)['vertex']
    pts, ix = np.unique(tris.reshape(-1,3), axis=0, return_inverse=True)
    solid = Manifold(Mesh(pts, ix.reshape(-1,3).astype(np.uint32)))
    if solid.status() != Error.NoError or solid.volume() <= 0:
        raise ValueError(f'Invalid stock mesh: {name}')
    return solid


def check_solid(solid):
    if solid.status() != Error.NoError or solid.volume() <= 0 or len(solid.decompose()) != 1:
        raise ValueError('Expected one connected, closed, positive-volume part')


def egg(center, radii):
    return Manifold.sphere(1, 20).scale(radii).translate(center)


def rounded_span(start, end, radii):
    return Manifold.batch_hull([egg(start,radii), egg(end,radii)])


def clip_section(stock, side):
    # Signed Y/Z section of the unmodified rigid lip. The active ±38 mm of
    # each lip is flat along X. Clip around the outer edge, in front of the tape.
    profile = stock.rotate((0,-90,0)).slice(.017).transform(((0,1,0),(-1,0,0)))
    inner = profile.offset(.15, JoinType.Round)
    wall = profile.offset(1.48, JoinType.Round) - inner
    y = FRONT[side]
    bottom = 9 if side == 'upper' else 0
    top = 14 if side == 'upper' else 8.8
    inward_top = 4.0 if side == 'upper' else 2.35
    inward_bottom = 1.80 if side == 'upper' else 4.90
    bound = CrossSection([[(y-3,bottom-2),(y+inward_bottom,bottom-2),
                           (y+inward_bottom,(top+bottom)/2),(y+inward_top,(top+bottom)/2),
                           (y+inward_top,top+2),(y-3,top+2)]])
    # End the locator on the outer face. Both mating faces of the jaws stay
    # clear even at full closure. The two side rails provide the squeeze.
    z0,z1 = (10.15,16) if side == 'upper' else (-2,7.6)
    outer = CrossSection([[(y-4,z0),(y+6,z0),(y+6,z1),(y-4,z1)]])
    return max((wall ^ bound ^ outer).decompose(), key=lambda section: section.area())


def clip_at(section, x):
    width = 6.4
    return section.extrude(width).transform(((0,0,1,x-width/2),(1,0,0,0),(0,1,0,0)))


def build(stock, side, teeth=True, tusks=False, preload=.10):
    if side not in FRONT or (tusks and side != 'upper'):
        raise ValueError('Tusks fit the upper cuff only')
    y = FRONT[side]
    upper = side == 'upper'
    z = 13.5 if upper else 2.6
    gum_radius = 2.35 if upper else 4.4
    # The width-spanning strip sits in front of one lip and carries no teeth.
    # Teeth and gums turn the corner and run fore-aft outside both cheeks.
    cheek_x = 45.8
    beam = rounded_span((-cheek_x,y-1.15,z),(cheek_x,y-1.15,z),(1.15,1.02,gum_radius))
    section = clip_section(stock,side)
    stations = (-23.0,23.0) if side == 'upper' else (-33.0,33.0)
    components = [beam] + [clip_at(section,x) for x in stations]
    for sign in (-1,1):
        x = sign*cheek_x
        components.append(rounded_span((x,y-1.15,z),(x,y+23.6,z),(1.15,1.25,gum_radius)))
        for reach in (8,20):
            contact_z = 12.5 if upper else 4.9
            contact_y = y+reach
            hits = stock.ray_cast((sign*52,contact_y,contact_z),(sign*35,contact_y,contact_z))
            if not hits:
                raise ValueError('Unable to locate rigid cheek for grip pad')
            wall_x = abs(hits[0].position[0])
            components.append(egg((sign*(wall_x+1.30-preload),contact_y,contact_z),(1.30,1.65,1.1)))
        if teeth:
            stations_y = (-1.2,6.3,13.8,21.3) if upper else (1.85,9.35,16.85,24.35)
            root_z = 12.2 if upper else 5.5
            tip_z = 7.25 if upper else 11.25
            for station in stations_y:
                roots = [egg((x,y+station+delta,root_z),(1.05,.85,1.05)) for delta in (-1.15,1.15)]
                # The longer upper row passes outside the lower cheek rail.
                tip = egg((x + sign*(3.2 if upper else 2.6),y+station,tip_z),(.90,.90,.90))
                components.append(Manifold.batch_hull([*roots,tip]))
    if tusks:
        for sign in (-1,1):
            root = egg((sign*32.5,y-2.3,z+1.0),(3.8,2.2,2.2))
            bend = egg((sign*34.0,y-10,z+5.2),(3.3,3.8,2.9))
            mid = egg((sign*30.8,y-32,z+5.6),(2.65,3.4,2.4))
            end = egg((sign*25.4,y-53,z+4.3),(2.3,2.1,2.1))
            components.extend(Manifold.batch_hull(pair) for pair in ((root,bend),(bend,mid),(mid,end)))
    solid = Manifold.batch_boolean(components, OpType.Add)
    if upper and not tusks:
        solid = solid.trim_by_plane((0,0,-1),-15.3)
    if not upper:
        solid = solid.trim_by_plane((0,0,1),-1.30)
    check_solid(solid)
    return solid


def bed_rotation(side, tusks=False):
    return (180,0,0) if side == 'upper' and not tusks else (0,0,0)


def print_pose(solid, tusks=False, side='upper'):
    # Standard cuffs bed the spine and both cheek rails on the same plane.
    part = solid.rotate(bed_rotation(side,tusks))
    lo,hi = np.array(part.bounding_box()).reshape(2,3)
    return part.translate((-(lo[0]+hi[0])/2,-(lo[1]+hi[1])/2,-lo[2]))


def write_stl(solid, filename):
    solid = solid.simplify(.008)
    check_solid(solid)
    m = solid.to_mesh()
    triangles = m.vert_properties[m.tri_verts,:3].copy()
    normals = np.cross(triangles[:,1]-triangles[:,0], triangles[:,2]-triangles[:,0])
    sizes = np.linalg.norm(normals,axis=1)
    if np.any(sizes<=0):
        raise ValueError(f'Degenerate triangles in {filename}')
    out = np.zeros(len(triangles), dtype=DTYPE)
    out['normal'] = normals / sizes[:,None]
    out['vertex'] = triangles
    filename.write_bytes(b'V5 TPU cuff, mm'.ljust(80,b' ') + struct.pack('<I',len(out)) + out.tobytes())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--preload',type=float,choices=(0,.10,.20),default=.10,
                        help='nominal rib interference in mm, selected using a coupon')
    args = parser.parse_args()
    args.output.mkdir(exist_ok=True,parents=True)
    for side, name in [('upper','HEAD_BOTTOM'),('lower','JAW')]:
        stock = read_stock(name)
        variants = [(True,False,'teeth')]
        if side == 'upper':
            variants += [(False,True,'tusks'),(True,True,'teeth-tusks')]
        for teeth,tusks,label in variants:
            item = build(stock,side,teeth,tusks,args.preload)
            write_stl(print_pose(item,tusks,side=side),args.output/f'{side}-{label}.stl')
            print(f'{side}-{label}: {item.volume()/1000:.3f} cm3')
        for pressure in (0,.10,.20):
            item = build(stock,side,False,False,pressure)
            write_stl(print_pose(item,side=side),args.output/f'{side}-fit-{pressure:.2f}.stl')


if __name__ == '__main__':
    main()
