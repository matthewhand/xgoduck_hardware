"""Make two-colour 3MF parts on the outward, bed-facing panels of the V5 STLs."""
import argparse
import hashlib
from pathlib import Path
import subprocess
from zipfile import ZipFile, ZIP_DEFLATED

import numpy as np
from manifold3d import CrossSection, Error, Manifold, Mesh, OpType

ROOT = Path(__file__).resolve().parents[2]
REV = "4496fad52041db1cad36ded6cc49d942e037b4af"
PARTS = {
    "HEAD_BOTTOM": (5.0, "69789f906e1298d3d3af367eea0c65d7e844c329648c181acb6f01176d462cb3"),
    "BODY_SHELL_LEFT": (2.5, "33d07c9bc364a6eea4487109ea9a45c1eedd62703ec7b9aa261edd689201d199"),
    "BODY_SHELL_RIGHT": (2.5, "961beeb1158a0883047b9d65747daf53f77a0db55f636d271a333ecd3fb47797"),
    "THIGH_LEFT_SHELL": (2.002, "95c9096d70feb4485f55ac6000241da777406d0f236cde2470d7350b95a5b958"),
    "THIGH_RIGHT_SHELL": (2.0, "2577a16ee6a6a87148a67385cf4a629cf637ea5ca2342cfcf98679b7573a3058"),
}
COLORS = {"feather": "#65C9A6", "orbit": "#EAB362", "chevron": "#E48F82"}
DTYPE = np.dtype([("normal", "<f4", 3), ("points", "<f4", (3, 3)), ("attribute", "<u2")])


def read_part(name, cache=None):
    rel = f"structure/V5_AUDIO_VIDEO_/V5_AUDIO_VIDEO_{name}.stl"
    source = cache / f"{name}.stl" if cache else ROOT / rel
    raw = source.read_bytes() if source.exists() else subprocess.check_output(["git", "-C", str(ROOT), "show", f"{REV}:{rel}"])
    if hashlib.sha256(raw).hexdigest() != PARTS[name][1]:
        raise ValueError(f"Unexpected upstream geometry for {name}")
    triangles = np.frombuffer(raw, dtype=DTYPE, offset=84)["points"]
    points, indices = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
    solid = Manifold(Mesh(points, indices.reshape(-1, 3).astype(np.uint32)))
    if solid.status() != Error.NoError:
        raise ValueError(f"Invalid source for {name}")
    return solid


def stroke(points, width=1.0):
    discs = [CrossSection.circle(width / 2, 12).translate(p) for p in points]
    return CrossSection.batch_boolean([CrossSection.batch_hull(pair) for pair in zip(discs, discs[1:])], OpType.Add)


def artwork(style):
    if style == "feather":
        paths = [stroke([(-2, -7), (-.8, -2), (0, 2), (1.7, 7.2)])]
        for y in (-4, -.5, 3):
            x = .25 * y
            paths += [stroke([(x, y), (x-2.8, y+1.6), (x-4.5, y+4)]),
                      stroke([(x, y), (x+2.4, y+.8), (x+4.3, y+2.8)])]
        spacing = (13, 19)
    elif style == "orbit":
        stars = [(-5, -3), (-1, 3), (3.5, 2), (5, 6)]
        paths = [stroke(stars, .9), stroke([(-5, -3), (.5, -5), (3.5, 2)], .9)]
        paths += [CrossSection.circle(r, 20).translate(p) for p, r in zip(stars, (1.3, 1.65, 1.0, 1.15))]
        paths += [(CrossSection.circle(2.1, 32) - CrossSection.circle(1.1, 28)).translate((4.5, -5))]
        spacing = (17, 16)
    else:
        paths = [stroke([(-5, -1+y), (0, 3+y), (5, -1+y)], 1.2) for y in (-3, .3)]
        paths.append(stroke([(-2, -5), (0, -3.4), (2, -5)], 1.1))
        spacing = (14, 12)
    motif = CrossSection.batch_boolean(paths, OpType.Add)
    return CrossSection.batch_boolean([
        motif.translate(((x + (y % 2) * .5) * spacing[0], y * spacing[1]))
        for x in range(-5, 6) for y in range(-6, 7)
    ], OpType.Add)


def decorate(stock, art, wall, finish, depth):
    # The outer face prints at Z=0. The inset mask keeps artwork off its
    # rounded edge, holes and slots. Only decorate solid wall below the inlay.
    panel = stock.slice(.02).offset(-2.5) ^ stock.slice(1.35).offset(-2)
    if finish == "relief":
        # Paired outward relief and inner recess require parallel, flat lands.
        panel ^= (stock.slice(wall-.02) - stock.slice(wall+.02)).offset(-2)
    pattern = art ^ panel
    # Remove fragments created where a stroke meets the boundary.
    pattern = CrossSection.batch_boolean([s for s in pattern.decompose() if s.area() >= 2], OpType.Add)
    if pattern.area() < 5:
        raise ValueError("No printable artwork in this panel")
    prism = pattern.extrude(depth + .02).translate((0, 0, -.02))
    accent = stock ^ prism
    base = stock - prism
    if finish == "relief":
        # A coloured column spans the inlay depth and the raised layer.
        # Remove an equal-volume column from the inner wall under the artwork.
        accent += pattern.extrude(depth).translate((0, 0, -depth))
        base -= pattern.extrude(depth + .02).translate((0, 0, wall-depth))
    elif finish == "engraved":
        accent = Manifold()
    if base.status() != Error.NoError or accent.status() != Error.NoError:
        raise ValueError("Invalid decorated mesh")
    change = base.volume() + accent.volume() - stock.volume()
    if finish != "engraved" and abs(change) > .02:
        raise ValueError(f"Volume balance failed: {change:.4f} mm3")
    return base, accent, change


def write_3mf(path, base, accent, color):
    children = [(2, "Shell", base, 0)]
    if not accent.is_empty():
        children.append((3, "Artwork", accent, 1))
    model_ns = "http://schemas.microsoft.com/3dmanufacturing/core/2015/02"
    with ZipFile(path, "w", ZIP_DEFLATED, compresslevel=3) as archive:
        archive.writestr("[Content_Types].xml", '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types"><Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/><Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/></Types>')
        archive.writestr("_rels/.rels", '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Target="/3D/3dmodel.model" Id="model" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>')
        with archive.open("3D/3dmodel.model", "w") as stream:
            def emit(markup):
                stream.write(markup.encode("utf-8"))
            emit(f'<model unit="millimeter" xml:lang="en-US" xmlns="{model_ns}"><resources><basematerials id="1"><base name="Shell" displaycolor="#384856"/><base name="Artwork" displaycolor="{color}"/></basematerials>')
            for obj_id, name, solid, material in children:
                mesh = solid.to_mesh()
                emit(f'<object id="{obj_id}" type="model" name="{name}" pid="1" pindex="{material}"><mesh><vertices>')
                emit("".join(f'<vertex x="{x:.7f}" y="{y:.7f}" z="{z:.7f}"/>' for x, y, z in mesh.vert_properties[:, :3]))
                emit('</vertices><triangles>')
                emit("".join(f'<triangle v1="{a}" v2="{b}" v3="{c}"/>' for a, b, c in mesh.tri_verts))
                emit('</triangles></mesh></object>')
            emit('<object id="4" type="model" name="V5 decorated shell"><components>')
            emit("".join(f'<component objectid="{i}"/>' for i, *_ in children))
            lift = -min(solid.bounding_box()[2] for _, _, solid, _ in children)
            emit(f'</components></object></resources><build><item objectid="4" transform="1 0 0 0 1 0 0 0 1 0 0 {lift:.7f}"/></build></model>')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--style", choices=COLORS, default="feather")
    parser.add_argument("--parts", nargs="+", choices=PARTS, default=list(PARTS))
    parser.add_argument("--finish", choices=["flush", "relief", "engraved"], default="flush")
    parser.add_argument("--depth", type=float, choices=(.2, .4), default=.4)
    parser.add_argument("--stock-dir", type=Path)
    args = parser.parse_args()
    art = artwork(args.style)
    args.output.mkdir(parents=True, exist_ok=True)
    for name in args.parts:
        stock = read_part(name, args.stock_dir)
        base, accent, change = decorate(stock, art, PARTS[name][0], args.finish, args.depth)
        path = args.output / f"{name.lower()}-{args.style}-{args.finish}.3mf"
        write_3mf(path, base, accent, COLORS[args.style])
        print(f"{path.name}: accent {accent.volume():.3f} mm3, stock delta {change:+.5f} mm3", flush=True)


if __name__ == "__main__":
    main()
