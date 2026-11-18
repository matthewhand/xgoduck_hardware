"""Render and check the proposed harness in the seated V5 enclosure STL frame (mm)."""
import json
from pathlib import Path
import runpy

import numpy as np
from manifold3d import Manifold

HERE = Path(__file__).resolve().parent
ROUTE = json.loads((HERE / "contact-harness.json").read_text(encoding="utf-8"))


def box(center, size):
    return Manifold.cube(tuple(size), center=True).translate(tuple(center))


def tube(a, b, radius):
    a, b = np.array(a), np.array(b)
    tube_axis = b-a
    length = np.linalg.norm(tube_axis)
    z = tube_axis/length
    ref = np.array([0, 0, 1]) if abs(z[2]) < .9 else np.array([0, 1, 0])
    x = np.cross(z, ref)
    x /= np.linalg.norm(x)
    transform = np.column_stack((x, np.cross(z, x), z, a))
    return Manifold.cylinder(length, radius, radius, 12).transform(transform)


def parked(points, amount):
    pivot = np.array(ROUTE["parking"]["pivot_mm"])
    theta = np.deg2rad(amount*ROUTE["parking"]["rotation_z_deg"])
    cs, sn = np.cos(theta), np.sin(theta)
    return (points-pivot)@np.array([[cs,sn,0],[-sn,cs,0],[0,0,1]]) + pivot


def parts(park=0):
    """Yield (record, manifold, RGB), with park=1 for the disconnected harness."""
    for item in ROUTE["parts"]:
        kind = item["kind"]
        x, y, z = item["center_mm"]
        size = item["size_mm"]
        solid = box([x,y,z], size)
        if kind == "strip":
            # 18 mm folding centres. The head side of each tab is under its wire.
            for dx in [-9,9]:
                solid += box([x+dx,y,z+1.4], [.5,size[1],2.4])
        if kind == "plug":
            # An unregistered pigtail envelope. There is no rendered board socket.
            solid = tube([x-size[0]/2,y,z], [x+size[0]/2,y,z], size[1]/2)
        if item["id"] in ROUTE["parking"]["moving_parts"] and park:
            pivot=np.array(ROUTE["parking"]["pivot_mm"])
            solid=solid.translate(-pivot).rotate((0,0,park*ROUTE["parking"]["rotation_z_deg"])).translate(pivot)
        colors = dict(strip=(238,188,83), isolator=(147,84,170), connector=(23,131,138),
                      plug=(73,76,84), relief=(101,110,116), mount=(155,163,171))
        color = (45,161,157) if item["id"] == "disconnect-carrier" else colors[kind]
        yield item, solid, color

    r=ROUTE["wire"]["jacket_od_mm"]/2
    for item in ROUTE["leads"]:
        pts=np.array(item["points_mm"], dtype=float)
        if park and "park_rotate_from_index" in item:
            k = item["park_rotate_from_index"]
            pts[k:] = parked(pts[k:], park)
        jacket = Manifold()
        for a,b in zip(pts,pts[1:]):
            jacket += tube(a,b,r)
        for xyz in pts[1:-1]:
            jacket += Manifold.sphere(r,12).translate(xyz)
        yield item, jacket, (215,73,65) if item["pole"]=="A" else (44,84,165)


def check():
    cad=runpy.run_path(str(HERE/"charging-pack.py"))
    fixed, cover, rigid, tooth = cad["build"](cad["stock_enclosure"](),.25)
    seated=list(parts())
    pack_wire=Manifold()
    for record, solid, _ in seated:
        if record.get("pole") and record["side"]=="pack":
            pack_wire += solid
        a=(solid^fixed).volume()
        b=(solid^cover).volume()
        print(record["id"], "vs fixed/cover mm3", round(a,4), round(b,4))
        if max(a,b)>1e-4:
            raise ValueError("Electrical routing overlaps a printed part")
    pack_electrics = sum((s for rec,s,_ in seated if rec["side"]=="pack"),Manifold())
    fixed_parked = sum((s for rec,s,_ in parts(1) if rec["side"]=="carrier"),Manifold())
    released=rigid+tooth.translate((0,0,-.85))
    housing_path=cad["sweep"](released,76)
    # The candidate components are entirely outside the stock enclosure while
    # moving headward except the pad exits, whose two insulated leads are swept.
    wiring_path=cad["sweep"](pack_wire,76)
    for name,other in [("fixed enclosure",fixed),("parked carrier harness",fixed_parked)]:
        overlap=(wiring_path^other).volume()
        print("Wire sweep vs",name,round(overlap,5))
        if overlap>1e-4:
            raise ValueError("Moving wiring collision")
    if (housing_path^fixed_parked).volume()>1e-4:
        raise ValueError("Parked harness blocks the cover")
    # For the remaining installed parts, sweep their boxes. These include the
    # folded ends, which extend in Z beyond the thin visible pad face.
    for rec,solid,_ in seated:
        if rec["side"]!="pack" or "points_mm" in rec: continue
        swept=cad["sweep"](solid,76)
        if max((swept^fixed).volume(),(swept^fixed_parked).volume())>1e-4:
            raise ValueError(f"Slide collision at {rec['id']}")
    # Inspect the two insulated paths as full jackets (including their corners).
    for side in ["carrier","pack"]:
        for frac in [0,1]:
            collection=list(parts(frac))
            groups=[sum((s for rec,s,_ in collection if rec.get("pole")==pole and rec["side"]==side),Manifold()) for pole in ["A","B"]]
            if (groups[0]^groups[1]).volume()>1e-4:
                raise ValueError(f"Wire jackets overlap on {side} at park={frac}")
    for fraction in np.linspace(0,1,9):
        carrier=sum((s for rec,s,_ in parts(fraction) if rec["side"]=="carrier"),Manifold())
        if (carrier^(fixed+cover+pack_electrics)).volume()>1e-4:
            raise ValueError(f"Cannot park connector at {fraction}")
    print("Clear through 76 mm with the carrier half parked.")


if __name__=="__main__":
    check()
