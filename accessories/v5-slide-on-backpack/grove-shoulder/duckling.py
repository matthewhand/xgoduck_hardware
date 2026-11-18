"""Rear-loading shoulder pocket for a housed Grove camera. All dimensions in mm."""
import argparse
from pathlib import Path
import runpy

import numpy as np
from manifold3d import Manifold as M

HERE = Path(__file__).resolve().parent
BASE = runpy.run_path(str(HERE / "carrier.py"))
box, route = BASE["box"], BASE["route"]
seated, stack = BASE["SEATED"], BASE["STACK"]
ANCHOR = np.array([-55., -48., 20.])


def oval(center, radii):
    return M.sphere(1, 40).scale(radii).translate(center)


def rounded_box(center, size, radius):
    core = M.cube(np.asarray(size)-2*radius, center=True)
    return core.minkowski_sum(M.sphere(radius,24)).translate(center)


def design(width=30.8, height=68.5, depth=28., lens_height=48.):
    """Local front plane is x=0. Insert the case from negative x until it seats."""
    if min(width, height, depth) < 10 or not height*.45 < lens_height < height-6:
        raise ValueError("Check the dimensions and lens centre height above the case base")
    clearance = .45
    half = width/2 + clearance
    socket = box(-depth-80,-half,-clearance,depth+80,2*half,height+2*clearance)
    lens_keepout = M.cylinder(45,8,8+45*np.tan(np.deg2rad(31)),64).rotate((0,90,0)).translate((0,0,lens_height))
    cable_slot = box(-depth-8,-7,-10,depth+5,14,17)
    pocket_h = lens_height-14
    pack = rounded_box((-depth/2,0,pocket_h/2-1), (depth+6,width+8,pocket_h+4),4)
    # Welting around the pouch and a rounded outside pocket on its right cheek.
    pack += rounded_box((-depth/2,0,pocket_h-3),(depth+7,width+9,5),2)
    pack += rounded_box((-depth/2,-half-2,pocket_h*.45),(depth*.65,6,pocket_h*.57),2.8)
    pack += rounded_box((2,0,pocket_h*.39),(7,width*.78,pocket_h*.57),3)
    pack += rounded_box((3.2,0,pocket_h*.60),(6.8,width*.82,5.4),2.5)
    # Strap slots open into the pocket near the rear. A removable band crosses
    # the back of the inserted case, leaving its front window and bottom lead free.
    for side in (-1,1):
        pack -= box(-depth+2, side*half-8, 11, 3,16,9)
        pack -= box(-depth-5, side*half-1.4, 11, 9,2.8,9)
    pack = (pack - socket) - cable_slot

    face = oval((-4,0,lens_height+1),(depth*.64,half+6,height-lens_height+4))
    for i in range(2):
        tuft = oval((0,0,0),(3,2.6,6-i)).rotate((0,-32-10*i,0))
        face += tuft.translate((-6-4*i,(-1)**i*2,height+2-i))
    eyes = M()
    for sign in (-1,1):
        eye = (7,sign*(half*.98),lens_height+6)
        eyes += oval(eye,(3.8,3.4,4.3))
    bill = oval((9,0,lens_height-19),(14,half*.73,3.8))
    bill += oval((8,0,lens_height-21.8),(11,half*.64,2.7))
    # Keep the face shallow in front of the glass so the optical port stays small.
    face = face.scale((.50,1,1))
    eyes = eyes.scale((.50,1,1))
    # Every printable colour is masked by the full insertion volume and optics.
    face = face - socket - lens_keepout
    bill = bill - socket - lens_keepout
    eyes = eyes - socket - lens_keepout
    details = {"bill":bill, "eyes":eyes}
    outer = pack + face + bill + eyes
    face -= bill+eyes
    pack -= face+bill+eyes
    def fitted(s):
        return seated(s.translate(ANCHOR))
    root = BASE["SHOULDER"]["ROOT"]
    support = route([root,root+[0,0,4],(-68,-38,18),ANCHOR+[-depth*.45,0,-3]],3.4)
    support += box(*(root+[-5.5,-3.2,-.65]),11,6.4,3.3)
    # Keep even the cassette neck outside the case's swept insertion envelope.
    body = (stack["cartridge"]()+seated(support)+fitted(pack))-fitted(socket)-fitted(cable_slot)
    solids = {"pack":body, "face":fitted(face), **{k:fitted(v) for k,v in details.items()}}
    joined = (stack["cartridge"]()+seated(support)+fitted(outer))-fitted(socket)-fitted(cable_slot)
    components = [s for s in joined.decompose() if abs(s.volume()) > 1e-6]
    assert len(components)==1 and all(v.volume()>0 for v in solids.values())
    whole = components[0].simplify(.002)
    assert max((whole ^ fitted(tool)).volume() for tool in (socket,lens_keepout)) < 1e-4
    case = rounded_box((-depth/2,0,height/2),(depth,width,height),2.4)
    lens = M.cylinder(1.2,4.8,4.8,40).rotate((0,90,0)).translate((-.3,0,lens_height))
    lead = route([(-depth+4,0,3),(-depth-3,0,1),(-depth-5,0,-5),(-depth-4,4,-9),(-depth+1,8,-11)],1.5)
    assert max((whole ^ fitted(v)).volume() for v in (case,lens,lead)) < .001
    visual = {"housing": fitted(case), "lens": fitted(lens), "lead": fitted(lead)}
    return whole, solids, visual


if __name__=="__main__":
    cli=argparse.ArgumentParser(description=__doc__)
    for field,default in [('width',30.8),('height',68.5),('depth',28.),('lens-height',48.)]:
        cli.add_argument('--'+field,type=float,default=default)
    whole, solids, _ = design(**vars(cli.parse_args()))
    path = HERE / 'duckling-pocket.stl'
    stack['tail']['write_stl'](path,stack['print_pose'](whole))
    runpy.run_path(str(HERE.parents[1]/'v5-dino-beak/verify-fit.py'))['check_export'](path)
    # The four parts must share one transformation for a multi-tool import.
    lift = -whole.rotate((90,0,0)).bounding_box()[2]
    colours = {"pack": solids["pack"], "head": solids["face"],
               "bill": solids["bill"], "eyes": solids["eyes"]}
    assert abs(sum(p.volume() for p in colours.values())-whole.volume()) < 1
    folder = HERE / "duckling-colours"
    folder.mkdir(exist_ok=True)
    write_parts = runpy.run_path(str(HERE.parent/"rail-fit.py"))["write_stl"]
    for name, solid in colours.items():
        pieces = [p.simplify(.0005) for p in solid.decompose() if p.volume()>1e-6]
        placed = M.compose(pieces).rotate((90,0,0)).translate((0,0,lift))
        target=folder / (name+".stl")
        write_parts(target,placed)
        raw = target.read_bytes()
        count = int.from_bytes(raw[80:84],"little")
        triangles = np.ndarray((count,3,3),dtype="<f4",buffer=raw,offset=96,strides=(50,12,4))
        _, indices = np.unique(triangles.reshape(-1,3),axis=0,return_inverse=True)
        faces = indices.reshape(-1,3)
        edges = np.concatenate([faces[:,[0,1]],faces[:,[1,2]],faces[:,[2,0]]])
        assert np.all(np.unique(np.sort(edges,axis=1),axis=0,return_counts=True)[1]==2), name
    print(f"Closed one-piece carrier, solid mass estimate {whole.volume()*.00124:.2f} g at 1.24 g/cm3")
