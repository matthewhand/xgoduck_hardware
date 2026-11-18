"""Export a bench coupon for a side-entry JBTN1 service key. Dimensions are mm.

The sprung target is a dummy. This fixture has no Duck or PCB mounting datum.
Y follows insertion, Z is up, and clockwise key rotation lowers the eccentric.
"""
import argparse
import importlib.util
from pathlib import Path

import numpy as np
from manifold3d import Manifold, OpType


def load_stl_writer():
    spec = importlib.util.spec_from_file_location("charging_pack", Path(__file__).with_name("charging-pack.py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.write_stl


def box(x0, x1, y0, y1, z0, z1):
    return Manifold.cube((x1-x0, y1-y0, z1-z0)).translate((x0, y0, z0))


AXIS_Z = 8.0
SEAT_Y = 0.0
MAX_TURN = 60  # Stop before the eccentric reaches dead centre.


def cylinder_y(radius, y0, y1, x=0):
    return (Manifold.cylinder(y1-y0, radius, circular_segments=72)
            .rotate((-90, 0, 0)).translate((x, y0, AXIS_Z)))


def turn(part, degrees):
    return part.translate((0, 0, -AXIS_Z)).rotate((0, degrees, 0)).translate((0, 0, AXIS_Z))


def build():
    shaft = cylinder_y(2.40, -2.3, 14.8)
    cam = cylinder_y(1.50, 14.7, 17.9, x=.85)
    lug = box(1.9, 4.4, 4.8, 6.4, 7.3, 8.7)
    # Broad tab and collar are both removed when the robot runs.
    handle = box(-10, 10, -6, -2, 6.2, 9.8)
    collar = cylinder_y(3.7, -2.3, SEAT_Y)
    key = shaft + cam + lug + collar + handle

    # The two bearings keep the key on axis. The enlarged sector admits
    # the lug only at the seated depth. Its ends limit rotation.
    bearing = cylinder_y(2.72, -.2, 14.4)
    front = box(-6, 6, 0, 8.0, 1, 14) - bearing
    straight = box(1.65, 4.72, -.2, 6.68, 7.02, 8.98)
    sector_lug = box(1.65, 4.72, 4.52, 6.68, 7.02, 8.98)
    sector = Manifold.batch_boolean([turn(sector_lug, a) for a in range(0, MAX_TURN + 1, 2)], OpType.Add)
    front = front - straight - sector
    rear = box(-5, 5, 11, 14.3, 1, 14) - bearing
    base = box(-18, 6, 0, 20, 0, 2)
    roof = box(-5, 5, 7.8, 20, 12, 14)
    end_wall = box(-5, 5, 18.9, 20, 1, 14)
    anchor = box(-18, -14.8, 13.6, 18.6, 1, 6.15)
    housing = base + front + rear + roof + end_wall + anchor
    leaf = box(-15, 2.2, 15, 17.7, 5.25, 6.15)
    return housing, leaf, key, cam


def verify(housing, leaf, key, cam):
    # The key occupies its final orientation throughout straight insertion.
    for distance in np.arange(-21, .01, .25):
        placed = key.translate((0, distance, 0))
        if (placed ^ (housing + leaf)).volume() > 1e-6:
            raise ValueError(f"Blocked neutral insertion at {distance} mm")
    for angle in range(MAX_TURN + 1):
        if (turn(key, angle) ^ housing).volume() > 1e-5:
            raise ValueError(f"Rigid interference at {angle} degrees")
    for angle in (-10, MAX_TURN + 10):
        if (turn(key, angle) ^ housing).volume() < .1:
            raise ValueError("Missing rotational stop")
    if (turn(key.translate((0, -2.4, 0)), 30) ^ housing).volume() < .1:
        raise ValueError("Key can turn before reaching its seat")
    if (turn(key, MAX_TURN).translate((0, -2.4, 0)) ^ housing).volume() < .1:
        raise ValueError("Key can be withdrawn while pressing")
    if (key.translate((0, .5, 0)) ^ housing).volume() < .1:
        raise ValueError("Missing insertion shoulder")
    if (turn(cam, MAX_TURN) ^ leaf).volume() < .1:
        raise ValueError("Eccentric misses dummy target")
    print(f"Neutral insertion and 0-{MAX_TURN} degree rotation clear the rigid fixture.")
    print("Partial insertion and both rotation stops engage. Dummy target displacement 0.39 mm nominal.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    housing, leaf, key, cam = build()
    verify(housing, leaf, key, cam)
    args.output.mkdir(exist_ok=True, parents=True)
    write_stl = load_stl_writer()
    write_stl(args.output / "duck-button-bench-guide.stl", housing + leaf)
    # Handle face on the bed, shaft axis upright.
    write_stl(args.output / "duck-button-thumb-key.stl", key.rotate((90, 0, 0)).translate((0, 0, 6)))


if __name__ == "__main__":
    main()
