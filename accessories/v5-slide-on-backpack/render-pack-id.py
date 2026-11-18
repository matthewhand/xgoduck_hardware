"""Two orthographic placement views of the optional backpack reader."""
from pathlib import Path
import runpy
import sys

from PIL import Image

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from cad_figure import bg, label, render


def mesh(solid, color):
    raw = solid.to_mesh()
    return raw.vert_properties[raw.tri_verts, :3].copy(), color


def main():
    nfc = runpy.run_path(str(HERE / "pack-id.py"))
    cad = nfc["cad"]
    fixed, cover, _, _ = cad["build"](cad["stock_enclosure"](), .25)
    parts = nfc["keepouts"]()
    shell, reader, antenna, tag = [178, 188, 185], [44, 158, 144], [215, 155, 62], [120, 88, 171]
    left = [mesh(fixed+nfc["reader_mount"](), shell), mesh(parts["reader"], reader),
            mesh(parts["antenna"], antenna)]
    right = [mesh(cover-nfc["tag_pocket"](), shell), mesh(parts["tag"], tag)]
    plate = Image.new("RGB", (940, 474), bg)
    for i, (items, axis, title) in enumerate(((left, [0, 0, -1], "Fixed enclosure"),
                                             (right, [0, 0, 1], "Inside the charging cover"))):
        x = i*470
        label(plate, (x+16, 8), title, 24, True)
        picture, _ = render(items, 470, 360, axis, [0, 1, 0])
        plate.paste(picture, (x, 48))
    label(plate, (16, 416), "Reader PCB", 19, True, tuple(reader))
    label(plate, (169, 416), "Antenna", 19, True, tuple(antenna))
    label(plate, (486, 416), "Passive tag", 19, True, tuple(tag))
    label(plate, (16, 448), "Opposing faces shown open. The tag aligns with the antenna when the cover slides home.", 16)
    plate.save(HERE/"figures/pack-id.png", optimize=True)


if __name__ == "__main__":
    main()
