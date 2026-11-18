# V5 slide-on backpack fit check

[Upstream comparison, assembly animation and rail sections](UPSTREAM-COMPARISON.md)

[Backpack identification and reader fit](PACK-ID.md) · [Charging connections](CHARGING-WIRING.md) · [Guarded side-entry button access](BUTTON-ACCESS.md)

The base is Luwu’s V5 audio/video print set at `4496fad52041db1cad36ded6cc49d942e037b4af`. Use `V5_AUDIO_VIDEO_ENCLOSURE_FOR_V1P1.stl` as the reference for a V1.1 board and `V5_AUDIO_VIDEO_ENCLOSURE_FOR_V1.stl` for a V1 board. The contact-backpack prototype below adds its rails directly to the V1.1 enclosure. The earlier perimeter-frame concept used the `V5_AUDIO_VIDEO_BATTERY.stl` interface as a mounting datum. Verify the actual screw lengths, engagement and clearances on the assembled Duck. Luwu shows the interface in step 23 of the [V5 assembly guide](https://wiki.xgorobot.com/kb/the-xgo-series-robot/duck-guide-06?model=duck).

Use the rails to constrain side and rear motion, two lower shoulders to carry the seated cover, and a releasable catch to retain the shroud in a fall. Keep the UNO Q and carrier fixed when removing the shroud. Check USB-C, the power controls, the wiring harness and the neck sweep during the full slide. The V1.1 carrier STEP has a 27.94 × 16.76 mm opening on the matrix face. Fit the window to that opening after checking the view with the actual boards mated. [Luwu V1.1 carrier STEP](https://github.com/LuwuDynamics/xgoduck_hardware/blob/4496fad52041db1cad36ded6cc49d942e037b4af/PCBA/Step/ArduinoUnoQV1.1.step) · [Arduino pinout](https://docs.arduino.cc/resources/pinouts/ABX00162-full-pinout.pdf)

The 18 mm tall coupon measures rail fit in the intended material and print orientation. Print both STLs as exported. The sockets have 0.15, 0.25 and 0.35 mm normal clearance per face, in order from the notched end. Try the loosest socket first and record which one slides by hand without obvious play. The coupon does not measure retention or locate the rails on the chassis.

Generate the two STLs with Python, NumPy and `manifold3d` installed.

```sh
python accessories/v5-slide-on-backpack/rail-fit.py path/to/output
```

## V1.1 slide-on contact backpack

`charging-pack.py` generates a replacement for `V5_AUDIO_VIDEO_ENCLOSURE_FOR_V1P1.stl` and a removable protective contact cover. The enclosure uses the original battery bracket and mounting hardware. Two outboard dovetails are integral to its sides. With the Duck upright, start the cover above the enclosure on the head side. Lower it toward the feet until both socket ends bear on the fixed lower shoulders and the tongue engages. To remove it, lift the tongue outward and slide the cover upward. The cover uses printed retention and carries both metal contacts. Park the fixed pigtail outboard before sliding the cover. The [cutaway and mass budget](UPSTREAM-COMPARISON.md#proposed-contact-harness-and-mass) show the proposed separate routes and removable connector.

```sh
python accessories/v5-slide-on-backpack/charging-pack.py path/to/output --clearance 0.25
```

Print the rail coupon first to select 0.15, 0.25 or 0.35 mm clearance per face in your filament. Export the enclosure and cover at 100% in mm. Both STLs have their intended bed face at Z = 0. The charging cover adds 3.2 mm behind the stock rear plane. The contact ears span 77.6 mm and the wider rail sockets span 84.0 mm. The nominal 0.25 mm pair exports as `duck-v5-v1p1-top-entry-rail-enclosure.stl` and `duck-v5-v1p1-top-entry-contact-backpack.stl`. Match the two top-entry parts. Check the slice around the latch slots and shallow contact recesses.

The two contact pockets are 56 mm apart, matching the pitch on the fixed-pad chair. Each pocket is 20.5 × 2.9 × 0.45 mm with two fold-through slots on 18 mm centres. Use inert strip samples to check how far the metal stands proud of the surface and whether both pads touch simultaneously. Inside, two lands sit 0.30 mm clear of the enclosure, with separate grooves for the leads. The carrier-facing apertures and side cable exit remain accessible for the first fit check. Verify the actual display sightline, button reach and plugged cable bends on the assembled Duck.

The generator checks for solid meshes, seated interference, the released cover's continuous 76 mm headward slide against the complete stock enclosure and rails, lower stop engagement with the latch released and interference with withdrawal while latched. These are rigid CAD checks. Use the first prints unpowered to measure thumb release, pull-off force, plastic deflection and dock seating. This revision targets V1.1 and fixed metal pads. Electrical charging, latch fatigue and walking and get-up with the printed mass require separate validation.

To reproduce the comparison figures, use the exported 0.25 mm pair, the pinned upstream V5 assembly and the matching Duck Lab walking model. The renderer uses NumPy, Numba, Pillow, SciPy, MuJoCo and manifold3d.

```sh
python accessories/v5-slide-on-backpack/harness-geometry.py
python accessories/v5-slide-on-backpack/render-assembly.py --assembly path/to/xgoduckV5.3mf --stls path/to/output --runtime path/to/xgoduck_runtime_arduino
```
