"""Draw two matched Duck views and the isolated necklace from the arm study's collision geometry."""
import argparse
from pathlib import Path
import runpy
import sys

import mujoco as mj
import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = Path(__file__).resolve().parent
STUDY = runpy.run_path(str(HERE / "arms-study.py"))


def face(size, bold=False):
    fonts = ("segoeuib.ttf", "DejaVuSans-Bold.ttf") if bold else ("segoeui.ttf", "DejaVuSans.ttf")
    for font in fonts:
        try:
            return ImageFont.truetype(font, size)
        except OSError:
            pass
    return ImageFont.load_default()


def panel(load_scene, mode, rig):
    model = load_scene(mj, lambda spec: STUDY["build_arms"](spec, "off" if mode == "bare" else "arms"))
    data = mj.MjData(model)
    for joint in range(rig.model.njnt):
        source = rig.model.joint(joint)
        target = model.joint(source.name)
        count = 7 if source.type[0] == mj.mjtJoint.mjJNT_FREE else 1
        a, b = int(source.qposadr[0]), int(target.qposadr[0])
        data.qpos[b:b+count] = rig.data.qpos[a:a+count]
    mj.mj_forward(model, data)
    options = mj.MjvOption()
    for i in range(model.ngeom):
        geom = model.geom(i)
        name = geom.name or ""
        if name.startswith(("arm_", "collar_")):
            model.geom_group[i] = 1
            model.geom_rgba[i] = ([.14,.83,.88,1] if name.startswith("collar_strip") else
                                  [.97,.49,.14,1] if name.startswith("arm_") else [.99,.78,.24,1])
        elif mode == "isolated":
            model.geom_group[i] = 5
    camera = mj.MjvCamera()
    trunk = model.body("trunk_base").id
    neck = model.body("neck").id
    trunk_rotation = data.xmat[trunk].reshape(3,3)
    heading = np.rad2deg(np.arctan2(trunk_rotation[1,0], trunk_rotation[0,0]))
    camera.azimuth = heading + (18 if mode == "isolated" else 228)
    camera.elevation = -34 if mode == "isolated" else -14
    if mode == "isolated":
        collar = model.body("study_collar").id
        camera.lookat[:] = data.xpos[collar] + [0,0,-.003]
        camera.distance = .115
        options.geomgroup[5] = 0
    else:
        camera.lookat[:] = data.xpos[neck] + [.011,0,.009]
        camera.distance = .275
    model.stat.meansize = .025
    with mj.Renderer(model, height=340, width=416) as renderer:
        renderer.update_scene(data, camera=camera, scene_option=options)
        return Image.fromarray(renderer.render())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runtime", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    sys.path.insert(0, str(args.runtime / "python"))
    from walk_sim import WalkSimulation, load_recovery_scene
    sim = STUDY["POSTURE"]["simulation"](WalkSimulation,0,12,-48,-65,True,.45)(.45,0)
    for _ in range(300):
        sim.tick()
    w, h = 416, 340
    padding, gutter = 20, 10
    result = Image.new("RGB",(2*padding+3*w+2*gutter,h+162),"#11191e")
    draw = ImageDraw.Draw(result)
    draw.text((padding,12),"Removable neck arms",font=face(32,True),fill="#f4f6f1")
    draw.text((padding,55),"V5  /  MuJoCo geometry  /  TPU fit concept",font=face(17),fill="#acc0bc")
    labels = [("bare","Bare neck","Original V5 geometry"),
              ("worn","Necklace fitted","Both forearms hang from the moving neck"),
              ("isolated","Detached necklace","Rear opening  /  two shoulder sockets")]
    for i,(mode,title,caption) in enumerate(labels):
        x = padding+i*(w+gutter)
        result.paste(panel(load_recovery_scene,mode,sim),(x,82))
        draw.text((x,82+h+6),title,font=face(20,True),fill="#f3f5ee")
        draw.text((x,82+h+36),caption,font=face(15),fill="#bccdc9")
    result.save(args.output,optimize=True)


if __name__ == "__main__":
    main()
