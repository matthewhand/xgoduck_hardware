# V5 screw-retained cheek teeth

Each cheek strip carries four rounded teeth and attaches to its own two exterior bosses. There are four independent strips, upper and lower on each side. The operator can remove any one while the other three remain mounted. Both stock TPU mouth pads retain their original mounting and gripping surfaces.

![Installed rows, exploded fastenings and a section through one captured nut](figures/screw-teeth.png)

## Which parts to print

For upper teeth, substitute `print/upper-jaw-with-bosses.stl` for the official V5 `HEAD_BOTTOM`. For lower teeth, substitute `print/lower-jaw-with-bosses.stl` for `JAW`. Each replacement preserves the pinned upstream geometry, including the upper part’s enclosed void, and adds two exterior bosses on each cheek. Install the desired `upper-…-four-teeth.stl` or `lower-…-four-teeth.stl` rows. The names `plus-x` and `minus-x` refer to the upstream part coordinate system. The wider rim faces the operator and the flat strip face meets the bosses.

Upper-only or lower-only builds need just that rigid replacement. The original friction-fit cuffs are a separate design. This screw version uses no front bridge, face plate, stock holes or drilling. The permanent change while running without rows is the protruding boss and its captured nut on each cheek. Use the original stock parts for the original roll-contact profile.

## Hardware and printing

Each installed row uses two M2 × 6 mm machine screws, length measured under the head. The design models a head up to 3.8 mm in diameter and 2.0 mm high, and an M2 hex nut 4.0 mm across flats by 1.6 mm thick. Measure the actual hardware. Four rows use eight screws and eight nuts in total. Prepare four nuts for every replacement host, even if some mounts will initially be empty.

* Keep the replacement host in its exported orientation. The captured nuts lie on edge with their threaded axes pointing laterally outward. Each internal pocket is 1.91 mm deep along the screw axis and 4.22 mm wide across the opposed nut flats. The completed pocket restrains rotation and withdrawal.
* In the slicer, pause immediately before the first layer that spans the tops of the nut pockets. In the supplied bed orientation their upper planes are Z = 15.56 mm for the upper host and Z = 5.51 mm for the lower host. Choose the actual layer using the sliced preview. Keep support and infill out of the four nut pockets and the blind screw holes. Inspect support under the protruding bosses.
* At the pause, insert one nut in each of the four pockets, flats against the parallel pocket walls. Verify it sits below the next extrusion path before resuming. The finished host encapsulates the nuts. A missing nut cannot be inserted through the smaller finished screw hole.
* Print the side strips as exported with their flat mating faces on the bed. A flexible filament is appropriate for the teeth. Start both screws by hand and seat each head in its recess. The nominal head face is 0.50 mm below the outer rim. A 6 mm screw passes through the 2.4 mm web, through the entire nut, and ends 0.5 mm before the bottom of the blind hole. A longer screw or taller head needs a new check.
* Test installation and jaw travel unpowered. The simulations of the earlier friction-fit teeth do not qualify these new fasteners or the added contact surfaces for walking or get-up. Weigh the installed assembly, check thread engagement and rim deformation, and repeat the motion tests with that layout.

## CAD checks

`screw-teeth.py` builds the hosts from the pinned stock meshes, checks virtual fit with the modelled fasteners, sweeps the registered jaw from 0° to 30° in 0.25° steps, then exports the bed-oriented parts. The hardware envelopes and the teeth are checked against the opposing jaw, pads, face and ToF housing. The nominal camera check uses the registered V5 lens, a 640 × 480 frame and an assumed 55° vertical field of view. It checks that the metal stays laterally outside this view. The real lens, printed tolerances and strength still require measurement.

```sh
python screw-teeth.py
python render-screw-teeth.py
```

The scripts require the same Python, manifold3d, NumPy and rendering dependencies as the parent V5 dino-beak assembly. Numerical results for this exact mesh revision are in [fit.json](fit.json).
