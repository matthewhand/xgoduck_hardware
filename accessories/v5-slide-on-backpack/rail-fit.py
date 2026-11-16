"""Export a short rail and three matching test sockets, in millimetres."""
import argparse
from pathlib import Path
import struct

from manifold3d import CrossSection, Error, JoinType, Manifold
import numpy as np


CLEARANCES = (0.15, 0.25, 0.35)
SOCKET_CENTRES = (-14, 0, 14)
HEIGHT = 18


def rail_profile():
    return CrossSection([[(-2.8, -0.2), (2.8, -0.2), (4, 2.6), (-4, 2.6)]])


def build():
    profile = rail_profile()
    # The base stands in for the fixed frame. The sockets slide along Z.
    rail = profile.extrude(HEIGHT) + Manifold.cube((14, 2.8, HEIGHT)).translate((-7, -2.8, 0))
    socket = Manifold.cube((43, 5.6, HEIGHT)).translate((-21.5, .2, 0))
    for x, clearance in zip(SOCKET_CENTRES, CLEARANCES):
        channel = profile.offset(clearance, JoinType.Miter).extrude(HEIGHT + 2)
        socket -= channel.translate((x, 0, -1))
    # The corner notch identifies the 0.15 mm end of the socket block.
    socket -= Manifold.cube((2, 2, HEIGHT + 2)).translate((-22, 4.6, -1))
    for solid in (rail, socket):
        if solid.status() != Error.NoError or len(solid.decompose()) != 1 or solid.volume() <= 0:
            raise ValueError('Expected a single printable solid')
    for x in SOCKET_CENTRES:
        for offset in (-HEIGHT / 2, 0, HEIGHT / 2):
            if (rail.translate((x, 0, offset)) ^ socket).volume() > 1e-6:
                raise ValueError('The nominal rail obstructs the sliding path')
    return rail, socket


def write_stl(path, solid):
    mesh = solid.to_mesh()
    triangles = mesh.vert_properties[mesh.tri_verts, :3]
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    normals /= np.linalg.norm(normals, axis=1)[:, None]
    with path.open('wb') as output:
        output.write(b'XGO Duck V5 slide fit sample, millimetres'.ljust(80, b'\0'))
        output.write(struct.pack('<I', len(triangles)))
        for normal, face in zip(normals, triangles):
            output.write(struct.pack('<12fH', *normal, *face.flat, 0))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output_directory', type=Path)
    args = parser.parse_args()
    rail, sockets = build()
    args.output_directory.mkdir(parents=True, exist_ok=True)
    write_stl(args.output_directory / 'duck-v5-rail-fit-rail.stl', rail)
    write_stl(args.output_directory / 'duck-v5-rail-fit-sockets.stl', sockets)


if __name__ == '__main__':
    main()
