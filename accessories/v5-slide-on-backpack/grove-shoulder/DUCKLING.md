# Duckling camera backpack

![Duckling cassette on the V5 with the Grove lens above the orange bill](duckling-pocket.png)

The Grove Vision AI V2 and its camera stay in their own case. Slide the case in from behind the little backpack, with the lens facing forward. The lower pocket takes its weight. The yellow cheeks and orange bill frame the camera, and a notch at the bottom leads the cable back to the robot. The complete cassette rides on the existing shoulder deck.

## Check the housing before printing

The lens looks through the **49.5 × 30.5 mm** face. USB-C projects **0.5 mm** from the **30.5 × 31.5 mm** end nearest the lens, giving **50.0 mm** total length over the socket. The cassette carries the long axis horizontally and puts that short end on the outside of the Duck's right shoulder. Case depth in the direction of view is **31.5 mm**.

The window uses an estimated lens centre **12 mm from the USB-C end** of the plastic and **15.25 mm above the base**. With a 4.8 mm lens radius, the CAD also clears the 62° cone at each corner of a **±2 mm horizontal and vertical** offset around this estimate. The exact socket centre and separate Grove lead in the image are illustrative. The CAD clears the entire short end for lateral access throughout rear insertion.

Slide the case in with the cassette off the robot. Thread a narrow soft band through the two floor slots near the inboard end and wrap it over the case, clear of both the lens and the USB-C end. The guide allows 0.45 mm per side of the bare housing, and a 14 mm notch provides a route for the Grove lead. Check the actual cable exit, plug grip and bend radius on the assembly.

## Print options

- **One filament.** Use `duckling-pocket.stl`.
- **Four tools.** Select all four STLs in `duckling-colours/` and import them as parts of one object, keeping their positions. Assign teal to the pack, yellow to the head, orange to the bill and dark filament to the eyes. Preview supports around the bill and rail before printing.

The CAD check verifies rear insertion of the case and its projecting side socket, and keeps a 62° cone clear ahead of the provisional lens centre. The single-piece and colour exports passed closed-edge checks. Solid printed volume corresponds to **21.25 g at 1.24 g/cm³**, excluding the housed Grove, cable, band, deck and pin. Loaded walking and recovery, the physical lens view and physical retention still need testing.

Run `python duckling.py --width 49.5 --height 30.5 --depth 31.5 --lens-from-usb-end 12 --lens-height 15.25` to regenerate the kit in millimetres. Supply the bare housing dimensions. The generator models the USB-C projection separately. `python render.py --duckling` renders the default dimensions on the V5. The [bare-board variant and electrical notes](https://github.com/matthewhand/xgoduck_hardware/blob/v5-slide-on-backpack/accessories/v5-slide-on-backpack/grove-shoulder/README.md) cover the exposed-camera cassette and the Grove interface.
