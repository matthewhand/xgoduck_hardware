"""Optional V1.1 enclosure and removable cover for NFC identification, in seated STL mm."""
import argparse
from pathlib import Path
import runpy

from manifold3d import Manifold

HERE = Path(__file__).resolve().parent
cad = runpy.run_path(str(HERE / "charging-pack.py"))
box = cad["block"]


def tag_pocket():
    # Peel-and-stick 6 x 17 mm Type 2 tag. The tag faces the reader.
    return box(-27.5, -20.5, -22, -4, -1.85, 0)


def reader_mount():
    # Antenna sticks to the outward land at Z=1; the two boards and their
    # leads remain on the fixed side as the backpack slides past.
    antenna_land = box(-29.6,-18.4,-26,0,1,2.25)
    for y in (-24,-4):
        antenna_land += box(-28,-20,y,y+2,2,6.8)
    # The reader PCB sits on insulating adhesive strips. The two short
    # shoulders locate its sides; both cable ends remain open.
    seat = Manifold()
    for y in (-24,-5):
        seat += box(-18.15,-1.75,y,y+2.4,4.85,6.6)
        for x in (-19.50,-1.60):
            seat += box(x,x+1.2,y,y+2.4,2.8,6.6)
    return antenna_land + seat


def keepouts():
    # Component thicknesses include the antenna's adhesive allowance.
    return {
        "antenna": box(-29,-19,-25.5,-.5,.65,1),
        "reader": box(-18.15,-1.75,-25.5,-.5,.35,4.75),
        "tag": box(-27,-21,-21.5,-4.5,-1.85,-1.15),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    stock = cad["stock_enclosure"]()
    fixed, pack, skin, tooth = cad["build"](stock,.25)
    fixed += reader_mount()
    pack -= tag_pocket()
    parts = keepouts()
    carrier = fixed + parts["reader"] + parts["antenna"]
    for name in ("reader","antenna"):
        if (parts[name] ^ fixed).volume() > .0001:
            raise ValueError(f"{name} intersects the mount")
    if (parts["tag"] ^ pack).volume() > .0001:
        raise ValueError("Tag overlaps the cover")
    released = (skin - tag_pocket()) + tooth.translate((0,0,-.85)) + parts["tag"]
    if (cad["sweep"](released,76) ^ carrier).volume() > .0001:
        raise ValueError("Installed electronics block the cover slide")
    if parts["reader"].min_gap(parts["antenna"],10) < .1:
        raise ValueError("PCB touches antenna envelope")
    harness = runpy.run_path(str(HERE / "harness-geometry.py"))["parts"]
    for rec, solid, _ in harness():
        path = cad["sweep"](solid, 76) if rec["side"] == "pack" else solid
        if (path ^ carrier).volume() > .0001:
            raise ValueError(f"Charging harness crosses the reader mount at {rec['id']}")
    for i in range(9):
        for rec, solid, _ in harness(i / 8):
            if rec["side"] == "carrier" and (solid ^ carrier).volume() > .0001:
                raise ValueError(f"Cannot park {rec['id']} with the reader fitted")
    print("Reader, antenna and tag clear the seated print and 76 mm slide.")
    print("Nominal tag-to-antenna face gap %.2f mm" % parts["tag"].min_gap(parts["antenna"],10))
    args.output.mkdir(exist_ok=True,parents=True)
    cad["write_stl"](args.output/"v5-nfc-fixed-enclosure.stl",fixed)
    cad["write_stl"](args.output/"v5-nfc-charging-cover.stl",pack.translate((0,0,-cad["FACE"])))


if __name__ == "__main__":
    main()
