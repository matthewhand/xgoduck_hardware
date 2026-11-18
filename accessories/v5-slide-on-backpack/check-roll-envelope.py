"""First contact against a plane for the seated pack subassembly, in STL mm."""
from pathlib import Path
import runpy

import numpy as np


def main():
    here = Path(__file__).resolve().parent
    cad = runpy.run_path(str(here / "charging-pack.py"))
    harness = runpy.run_path(str(here / "harness-geometry.py"))
    fixed, cover, _, _ = cad["build"](cad["stock_enclosure"](), .25)
    meshes = {"fixed enclosure": fixed, "cover": cover}
    meshes.update((rec["id"], solid) for rec, solid, _ in harness["parts"]())
    vertices = {name: solid.to_mesh().vert_properties[:, :3] for name, solid in meshes.items()}
    directions = {"back": (0, 0, -1), "right": (1, 0, 0), "right/back": (1, 0, -1),
                  "left/back": (-1, 0, -1), "headward": (0, 1, 0)}
    for name, vector in directions.items():
        normal = np.array(vector) / np.linalg.norm(vector)
        extent = {part: float((xyz @ normal).max()) for part, xyz in vertices.items()}
        leader = max(extent, key=extent.get)
        print(f"{name:10s} {leader:24s} {extent[leader]:6.2f} mm, "
              f"{extent[leader] - extent['cover']:5.2f} mm beyond cover")


if __name__ == "__main__":
    main()
