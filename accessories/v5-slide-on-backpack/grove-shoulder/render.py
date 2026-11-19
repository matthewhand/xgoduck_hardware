"""Render the Grove cassette and nominal cable routes fitted to the V5 walking mesh."""
from pathlib import Path
import runpy
import argparse

import mujoco as mj
import numpy as np
from PIL import Image, ImageDraw

HERE = Path(__file__).resolve().parent
part = runpy.run_path(str(HERE / "carrier.py"))
base = runpy.run_path(str(HERE.parent / "render-shoulder-pointer.py"))
check = base["CHECK"]
colours = {"carrier": (.98,.52,.14), "deck": (.35,.42,.45), "pcb": (.10,.42,.80),
           "camera": (.1,.55,.27), "lens": (.12,.13,.16), "grove_lead": (.12,.93,.75),
           "csi_ribbon": (1,.76,.32), "grove_socket": (.94,.94,.88), "csi_socket": (.21,.23,.26)}


def configure(spec, duckling=False):
    check["add_stack"](spec, "removed")
    for g in spec.geoms:
        if g.name.startswith("stack_"):
            g.group = 5
    body = spec.body("trunk_base")
    palette = dict(colours)
    if duckling:
        _, solids, outlines = runpy.run_path(str(HERE/"duckling.py"))["design"]()
        palette.update(pack=(.06,.55,.54), face=(1,.79,.25), eyes=(.075,.042,.03),
                       bill=(1,.38,.08), housing=(.20,.24,.26),
                       usb_socket=(.76,.78,.8), lead=(.38,.82,.73))
    else:
        carrier, outlines = part["assembly"]()
        solids={"carrier":carrier}
    for name, solid in dict(solids,deck=part["STACK"]["deck"](),**outlines).items():
        mesh = solid.to_mesh()
        spec.add_mesh(name=name, uservert=check["trunk"](mesh.vert_properties[:,:3]).ravel(),
                      userface=mesh.tri_verts.ravel())
        body.add_geom(name=name, type=mj.mjtGeom.mjGEOM_MESH,meshname=name,
                      mass=0,contype=0,conaffinity=0,group=1,rgba=(*palette[name],1))


def main():
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--duckling",action="store_true")
    duckling=cli.parse_args().duckling
    runtime = HERE.parents[3] / "xgoduck_runtime_arduino"
    sim = runpy.run_path(str(runtime / "tools/check-duck-hardware.py"))["WalkSimulation"](0,0,configure=lambda s:configure(s,duckling))
    mj.mj_forward(sim.model,sim.data)
    canvas = Image.new("RGB",(900,382),"#172126")
    draw = ImageDraw.Draw(canvas)
    with mj.Renderer(sim.model,height=330,width=450) as renderer:
        views = ([((-.025,-.025,.04),.42,143,-12), ((-.075,-.048,.038),.18,130,-14)] if duckling
                 else [((0,-.012,.045),.46,118,-14), ((-.023,-.058,.050),.188,95,-26)])
        for i,(target,dist,az,el) in enumerate(views):
            camera=mj.MjvCamera()
            camera.lookat[:]=sim.data.xpos[sim.trunk]+sim.data.xmat[sim.trunk].reshape(3,3)@target
            camera.distance,camera.azimuth,camera.elevation=dist,az,el
            renderer.update_scene(sim.data,camera=camera)
            canvas.paste(Image.fromarray(renderer.render()),(i*450,0))
    captions = (("Duckling backpack","Camera window above the bill") if duckling else
                ("Grove Vision AI V2  ·  Right shoulder","Grove lead (mint)    CSI ribbon (gold)"))
    for i,label in enumerate(captions):
        draw.text((12+450*i,340),label,font=base["font"](18,True),fill="white")
    canvas.save(HERE / ("duckling-pocket.png" if duckling else "grove-shoulder.png"))


if __name__=="__main__":
    main()
