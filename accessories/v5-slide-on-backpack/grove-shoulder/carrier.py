"""Grove Vision AI V2 right-shoulder cassette, trunk axes in mm (+X forward)."""
from pathlib import Path
import json
import runpy

import numpy as np
from manifold3d import Manifold as M

HERE = Path(__file__).resolve().parent
SHOULDER = runpy.run_path(str(HERE.parent / "shoulder-pointer.py"))
STACK = runpy.run_path(str(HERE.parent / "stack/stack.py"))
SEATED = SHOULDER["seated"]
bridge = SHOULDER["bridge"]
PITCH = 159.648055 - 143.102947  # hole centres in Seeed's V2 KiCad Edge.Cuts
BOARD_X, BOARD_Y, BOARD_Z = -9., -60., 55.


def box(x, y, z, dx, dy, dz):
    return M.cube((dx, dy, dz)).translate((x, y, z))


def route(points, radius):
    return SHOULDER["union"]([bridge(a, b, radius) for a, b in zip(points, points[1:])])


def assembly():
    root = SHOULDER["ROOT"]
    points = [root, root+[0,0,4], (-64,-42,24), (-39,-60,38),
              (BOARD_X,BOARD_Y,42), (14,-60,44), (23,-60,50)]
    stem = route(points, 3.1) + box(*(root+[-5.5,-3.2,-.65]), 11, 6.4, 3.3)
    # Narrow crosspiece under PCB. Components on the bottom face overhang open air.
    stem += box(BOARD_X-2.6, BOARD_Y-11, 40, 5.2, 22, 3)
    pcb = box(BOARD_X-22.400868, BOARD_Y-10, BOARD_Z, 40, 20, 1.1962)
    for sign in (-1,1):
        y = BOARD_Y + sign*PITCH/2
        stem += M.cylinder(12, 2.5, 1.8, 40).translate((BOARD_X,y,43))
        hole = M.cylinder(22, 1.13, 1.13, 32).translate((BOARD_X,y,37))
        stem -= hole
        pcb -= M.cylinder(3,1.09,1.09,32).translate((BOARD_X,y,54.8))
    # Open 25.6 mm camera seat. Two vertical slots accept a narrow reusable band.
    front = box(24,-76,50, 2.6,32,26)
    front -= box(23,-70.5,53.5, 5,21,19)
    for y in (-75.3,-46.5):
        front -= box(23,y,68, 5,1.8,5)
    front += box(24,-74,49.2, 6.4,28,2)
    for y in (-74.4,-47.2):
        front += box(25.5,y,50.3, 4.8,1.6,5)
    stem += front
    # Side-loading guides accept a terminated cable without passing its plug.
    for x,y,z in [(-42,-59,38),(-63,-42,24)]:
        clip = box(x-2,y-7,z-1.5, 4,14,10)
        clip -= box(x-3,y-4.5,z+2, 6,9,4.7)
        clip -= box(x-3,y-8,z+2.3, 6,5,3.8)
        stem += clip
    cam = box(26.6,-72.5,51.2,1.3,25,23)
    lens = M.cylinder(7.7,3.5,3.5,32).rotate((0,90,0)).translate((27.9,-60,63))
    grove_path = [(-31.4,-60,51),(-36,-60,49),(-42,-59,42.5),
                  (-52,-52,36),(-63,-42,28.5),(-69,-38,24),(-76,-33,20)]
    lead = route(grove_path,1.55)
    stem -= route(grove_path,2.15)
    # Flat schematic ribbon envelope from forward CSI socket to back of camera.
    ribbons = [box(x,y-7,z,0.7,14,0.4) for x,y,z in
               [(5,-60,57),(12,-60,58),(19,-60,63),(24,-60,63),(26.6,-60,62)]]
    ribbon = SHOULDER["union"]([M.batch_hull([a,b]) for a,b in zip(ribbons,ribbons[1:])])
    cartridge = STACK["cartridge"]()
    beam = SEATED(stem)
    carrier = cartridge + beam
    assert (beam ^ cartridge).volume() > 20 and len(carrier.decompose()) == 1
    assert max((stem ^ part).volume() for part in (pcb,cam,lens,lead,ribbon)) < .01
    grove_socket = box(-31.4,-66,48.4,8,12,6.6)
    csi_socket = box(3,-69,56.2,4,18,1.6)
    assert max((stem ^ part).volume() for part in (grove_socket,csi_socket)) < .01
    return carrier, {name:SEATED(part) for name,part in
                     dict(pcb=pcb,camera=cam,lens=lens,grove_lead=lead,csi_ribbon=ribbon,
                          grove_socket=grove_socket,csi_socket=csi_socket).items()}


def main():
    carrier,_ = assembly()
    path = HERE / "grove-shoulder-cassette.stl"
    STACK["tail"]["write_stl"](path,STACK["print_pose"](carrier))
    runpy.run_path(str(HERE.parents[1]/"v5-dino-beak/verify-fit.py"))["check_export"](path)
    print(json.dumps(dict(printed_carrier_g=round(carrier.volume()*.00124,2),
                          mount_pitch_mm=PITCH, bounds_mm=carrier.bounding_box())))


if __name__ == "__main__":
    main()
