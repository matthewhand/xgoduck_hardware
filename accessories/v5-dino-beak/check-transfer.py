"""Screen closed-mouth head attachments against the stock Duck Lab policies."""
import argparse
import json
from pathlib import Path
import runpy

import numpy as np

HERE = Path(__file__).resolve().parent


def mass_boxes(exporter, teeth, tusks, density):
    cad = exporter["CAD"]
    result = []
    for side, part in (("upper", "HEAD_BOTTOM"), ("lower", "JAW")):
        if side == "lower" and not teeth:
            continue
        mesh = cad["build"](cad["read_stock"](part), side, teeth, tusks and side == "upper")
        transform = np.column_stack((exporter["PRINT_TO_HEAD"], exporter["OFFSET"][side]))
        solid = mesh.transform(transform)
        left, _, _, right, _, _ = solid.bounding_box()
        # Divide the actual mesh across its width, keeping the forward tips
        # and the two rearward cheek rows separate from the centre bridge.
        limits = np.linspace(left - .001, right + .001, 13)
        for i, (x0, x1) in enumerate(zip(limits, limits[1:])):
            piece = solid.trim_by_plane((1, 0, 0), x0).trim_by_plane((-1, 0, 0), -x1)
            vol = piece.volume()
            if vol < .001:
                continue
            lo, hi = np.array(piece.bounding_box()).reshape(2, 3)
            result.append(dict(name=f"{side}-{i}", parent="jaw_soft", mass_g=vol*density/1000,
                               center_m=((hi+lo)/2000).tolist(), size_m=((hi-lo)/1000).tolist()))
    return result


def key_clearance(meshes):
    # Compare first contact with a flat plane. The local -Z axis points out of
    # the beak and +Y points up. No key dynamics, servo load or IK are implied.
    stock = np.concatenate([v for _, _, variant, v, _ in meshes if variant == "all"])
    cuff = next(v for name, _, _, v, _ in meshes if name == "upper-tusks")
    rows = []
    for side in (-1, 1):
        half = cuff[cuff[:, 0]*side > 0]
        apex = half[np.argmin(half[:, 2])]
        tip = np.linalg.norm(cuff-apex, axis=1) <= 5.0
        others = np.vstack((stock, cuff[~tip]))
        for forward in (0, 30, 45, 60, 75, 90):
            for bank in (0, 5, 10, 15):
                a, b = np.deg2rad([forward, bank])
                n = [side*np.sin(b), -np.cos(a)*np.cos(b), -np.sin(a)*np.cos(b)]
                margin = float((cuff[tip]@n).max() - (others@n).max())
                rows.append(dict(side=side, forward_deg=forward, lateral_deg=bank,
                                 first_contact_margin_mm=round(margin, 2)))
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runtime", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    exp = runpy.run_path(str(HERE / "export-preview.py"))
    checks = runpy.run_path(str(args.runtime / "tools/check-duck-hardware.py"))
    cases = ("walk", "turn-left", "turn-right", "back", "front", "left", "right")
    data = {}
    for name, teeth, tusks in (("stock", False, False), ("teeth", True, False),
                               ("tusks", False, True), ("combined", True, True)):
        modules = mass_boxes(exp, teeth, tusks, 1.20) if teeth or tusks else []
        trials = {}
        for case in cases:
            tr = checks["run_case"](case, modules, {})
            reference = data["stock"]["cases"][case] if name != "stock" else tr
            tr["pass"] = bool(checks["passes"](case, tr, reference))
            trials[case] = tr
        data[name] = dict(added_mass_g=round(sum(m["mass_g"] for m in modules), 4), cases=trials)
        print(name, [c for c in cases if not trials[c]["pass"]], flush=True)
    plane = key_clearance(list(exp["meshes"](args.runtime)))
    args.output.write_text(json.dumps(dict(
        conditions="One nominal, deterministic run per case, stock walk/get-up ONNX, closed jaw, 1.20 g/cm3 TPU estimate. Twelve rigid floor-contact boxes per cuff. No hardware qualification.",
        motion=data, plane=plane), indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
