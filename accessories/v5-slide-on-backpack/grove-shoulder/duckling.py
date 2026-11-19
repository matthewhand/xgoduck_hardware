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
CASE_WIDTH, CASE_HEIGHT, CASE_DEPTH = 31.5, 30.5, 49.5
USB_PROJECTION = .5


def oval(center, radii):
    return M.sphere(1, 40).scale(radii).translate(center)


def rounded_box(center, size, radius):
    core = M.cube(np.asarray(size)-2*radius, center=True)
    return core.minkowski_sum(M.sphere(radius,24)).translate(center)


def design(width=CASE_WIDTH, height=CASE_HEIGHT, depth=CASE_DEPTH, lens_height=None):
    """Local front plane is x=0. Insert the case from negative x until it seats."""
    if lens_height is None:
        lens_height = height/2
    if min(width, height, depth) < 10 or not height*.40 < lens_height < height-6:
        raise ValueError("Check the dimensions and lens centre height above the case base")
    clearance = .45
    body_depth = depth
    depth += USB_PROJECTION
    half = width/2 + clearance
    socket = box(-depth-80,-half,-clearance,depth+80,2*half,height+2*clearance)
    lens_keepout = M.cylinder(45,8,8+45*np.tan(np.deg2rad(31)),64).rotate((0,90,0)).translate((0,0,lens_height))
    cable_slot = box(-depth-8,-7,-10,depth+5,14,17)
    pocket_h = max(8.,lens_height*.64)
    pack = rounded_box((-depth/2,0,pocket_h/2-1), (depth+6,width+8,pocket_h+4),4)
    # Welting around the pouch and a rounded outside pocket on its right cheek.
    pack += rounded_box((-depth/2,0,pocket_h-3),(depth+7,width+9,5),2)
    pack += rounded_box((-depth/2,-half-2,pocket_h*.45),(depth*.65,6,pocket_h*.60),2)
    pack += rounded_box((2,0,pocket_h*.39),(7,width*.78,pocket_h*.67),2.4)
    pack += rounded_box((3.2,0,pocket_h*.60),(6.8,width*.82,5.4),2.5)
    # Strap slots open into the pocket near the rear. A removable band crosses
    # the back of the inserted case, leaving its front window and bottom lead free.
    for side in (-1,1):
        pack -= box(-depth+2, side*half-8, 1.5, 3,16,min(9,pocket_h-3))
        pack -= box(-depth-5, side*half-1.4, 1.5, 9,2.8,min(9,pocket_h-3))
    pack = pack - socket - cable_slot - lens_keepout

    face = oval((-4,0,lens_height+1),(18,half+6,height-lens_height+4))
    for i in range(2):
        tuft = oval((0,0,0),(3,2.6,6-i)).rotate((0,-32-10*i,0))
        face += tuft.translate((-6-4*i,(-1)**i*2,height+2-i))
    eyes = M()
    for sign in (-1,1):
        eye = (7,sign*(half*.98),lens_height+6)
        eyes += oval(eye,(3.8,3.4,4.3))
    bill_height = max(1.5,lens_height-19)
    bill = oval((6,0,bill_height),(10,half*.73,3.3))
    bill += oval((5,0,bill_height-2.2),(8.5,half*.64,2.4))
    # Keep the face shallow in front of the glass so the optical port stays small.
    face = face.scale((.50,1,1))
    eyes = eyes.scale((.50,1,1))
    # Every printable colour is masked by the full insertion volume and optics.
    trim = socket + lens_keepout + cable_slot
    outer = (pack + face + bill + eyes) - trim
    pack -= face + bill + eyes
    face = face - bill - eyes - trim
    details = {"bill":bill-trim, "eyes":eyes-trim}
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
    case = rounded_box((-body_depth/2,0,height/2),(body_depth,width,height),2.4)
    lens = M.cylinder(1.2,4.8,4.8,40).rotate((0,90,0)).translate((-.3,0,lens_height))
    # Reserve the complete far end for the socket, irrespective of its lateral position.
    rear_end = box(-depth,-width/2,0,USB_PROJECTION,width,height)
    usb = rounded_box((-body_depth-USB_PROJECTION/2,0,height/2),(.5,9,3.5),.2)
    # Illustrative separate Grove lead. Its actual exit is still to be measured.
    lead = route([(-body_depth,0,5),(-depth-4,0,5),(-depth-9,1,2),(-depth-10,4,-4)],1.5)
    assert max((whole ^ fitted(v)).volume() for v in (case,lens,lead,rear_end)) < .001
    visual = {"housing": fitted(case), "lens": fitted(lens), "lead": fitted(lead), "usb_socket":fitted(usb)}
    return whole, solids, visual


if __name__=="__main__":
    cli=argparse.ArgumentParser(description=__doc__)
    for field,default in [('width',CASE_WIDTH),('height',CASE_HEIGHT),('depth',CASE_DEPTH),('lens-height',None)]:
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
