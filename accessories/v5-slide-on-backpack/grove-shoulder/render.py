"""Render the Grove cassette and nominal cable routes fitted to the V5 walking mesh."""
from pathlib import Path
import runpy

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


def configure(spec):
    check["add_stack"](spec, "removed")
    for g in spec.geoms:
        if g.name.startswith("stack_"):
            g.group = 5
    body = spec.body("trunk_base")
    carrier, outlines = part["assembly"]()
    for name, solid in dict(carrier=carrier,deck=part["STACK"]["deck"](),**outlines).items():
        mesh = solid.to_mesh()
        spec.add_mesh(name=name, uservert=check["trunk"](mesh.vert_properties[:,:3]).ravel(),
                      userface=mesh.tri_verts.ravel())
        body.add_geom(name=name, type=mj.mjtGeom.mjGEOM_MESH,meshname=name,
                      mass=0,contype=0,conaffinity=0,group=1,rgba=(*colours[name],1))


def main():
    runtime = HERE.parents[3] / "xgoduck_runtime_arduino"
    sim = runpy.run_path(str(runtime / "tools/check-duck-hardware.py"))["WalkSimulation"](0,0,configure=configure)
    mj.mj_forward(sim.model,sim.data)
    canvas = Image.new("RGB",(900,382),"#172126")
    draw = ImageDraw.Draw(canvas)
    with mj.Renderer(sim.model,height=330,width=450) as renderer:
        for i,(target,dist,az,el) in enumerate([((0,-.012,.045),.46,118,-14), ((-.023,-.058,.050),.188,95,-26)]):
            camera=mj.MjvCamera()
            camera.lookat[:]=sim.data.xpos[sim.trunk]+sim.data.xmat[sim.trunk].reshape(3,3)@target
            camera.distance,camera.azimuth,camera.elevation=dist,az,el
            renderer.update_scene(sim.data,camera=camera)
            canvas.paste(Image.fromarray(renderer.render()),(i*450,0))
    draw.text((12,340),"Grove Vision AI V2  ·  Right shoulder",font=base["font"](18,True),fill="white")
    draw.text((462,340),"Grove lead (mint)    CSI ribbon (gold)",font=base["font"](18,True),fill="white")
    canvas.save(HERE / "grove-shoulder.png")


if __name__=="__main__":
    main()
