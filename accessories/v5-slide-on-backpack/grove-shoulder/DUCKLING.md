# Duckling camera backpack

![Duckling cassette on the V5 with the Grove lens above the orange bill](duckling-pocket.png)

The Grove Vision AI V2 and its camera stay in their own case. Slide the case in from behind the little backpack, with the lens facing forward. The lower pocket takes its weight. The yellow cheeks and orange bill frame the camera, and a notch at the bottom leads the cable back to the robot. The complete cassette rides on the existing shoulder deck.

## Check the housing before printing

The owner's approximate dimensions are interpreted in the requested height × width × depth order: **30.5 × 31.5 × 49.5 mm**, with **0.5 mm** extra for the projecting USB-C socket on the last axis. The cassette reserves **50.0 mm** along that axis. The preview assumes the socket is at the open rear and the lens is centred on the opposite face, **15.25 mm above the base**. Check those orientations and the actual lens centre before printing. The rounded housing, connector position and cable in the render are illustrative.

The guide is **32.4 mm wide** and allows 0.45 mm per side. It has two slots for a narrow soft retaining band and a 14 mm wide cable notch. Slide the case in with the cassette off the robot and pass the band across the open back clear of the ports. Reconnect the lead and leave slack at the backpack end. The clearance check covers the protruding socket. The dimensions of a mated USB-C plug, its cable bend and the Grove lead exit still need a physical check.

## Print options

- **One filament.** Use `duckling-pocket.stl`.
- **Four tools.** Select all four STLs in `duckling-colours/` and import them as parts of one object, keeping their positions. Assign teal to the pack, yellow to the head, orange to the bill and dark filament to the eyes. Preview supports around the bill and rail before printing.

The CAD check verifies a continuous rear insertion path for the rectangular case envelope, clears the full 0.5 mm rear extension and keeps a 62° cone clear ahead of the nominal lens plane. The single-piece and colour exports passed closed-edge checks. Solid printed volume corresponds to **23.99 g at 1.24 g/cm³**, excluding the housed Grove, cable, band, deck and pin. Loaded walking and recovery, the physical lens view and physical retention still need testing.

Run `python duckling.py --width 31.5 --height 30.5 --depth 49.5 --lens-height 15.25` to regenerate the kit in millimetres. Supply the bare housing depth. The generator adds the socket projection once and writes the STL and colour bodies in this directory. `python render.py --duckling` renders the default dimensions on the V5. The [bare-board variant and electrical notes](https://github.com/matthewhand/xgoduck_hardware/blob/v5-slide-on-backpack/accessories/v5-slide-on-backpack/grove-shoulder/README.md) cover the exposed-camera cassette and the Grove interface.
