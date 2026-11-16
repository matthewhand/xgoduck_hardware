# V5 slide-on backpack fit check

The base is Luwu’s V5 audio/video print set at `4496fad52041db1cad36ded6cc49d942e037b4af`. Use `V5_AUDIO_VIDEO_ENCLOSURE_FOR_V1P1.stl` as the reference for a V1.1 board and `V5_AUDIO_VIDEO_ENCLOSURE_FOR_V1.stl` for a V1 board. The replacement is a fixed perimeter frame with two vertical rails and a removable shroud with matching sockets. Locate the frame from the `V5_AUDIO_VIDEO_BATTERY.stl` interface and route loads into the battery bracket and chassis through rigid spacers. Verify the actual screw lengths, engagement and clearances on the assembled Duck. Luwu shows the interface in step 23 of the [V5 assembly guide](https://wiki.xgorobot.com/kb/the-xgo-series-robot/duck-guide-06?model=duck).

Use the rails to constrain side and rear motion, a bottom stop to set the installed height, and a releasable catch to retain the shroud in a fall. Keep the UNO Q and carrier fixed when removing the shroud. Check USB-C, the power controls, the wiring harness and the neck sweep during the full slide. The V1.1 carrier STEP has a 27.94 × 16.76 mm opening on the matrix face. Fit the window to that opening after checking the view with the actual boards mated. [Luwu V1.1 carrier STEP](https://github.com/LuwuDynamics/xgoduck_hardware/blob/4496fad52041db1cad36ded6cc49d942e037b4af/PCBA/Step/ArduinoUnoQV1.1.step) · [Arduino pinout](https://docs.arduino.cc/resources/pinouts/ABX00162-full-pinout.pdf)

The 18 mm tall coupon measures rail fit in the intended material and print orientation. Print both STLs as exported. The sockets have 0.15, 0.25 and 0.35 mm normal clearance per face, in order from the notched end. Try the loosest socket first and record which one slides by hand without obvious play. The coupon does not measure retention or locate the rails on the chassis.

Generate the two STLs with Python, NumPy and `manifold3d` installed.

```sh
python accessories/v5-slide-on-backpack/rail-fit.py path/to/output
```
