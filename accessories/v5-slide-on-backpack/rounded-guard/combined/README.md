# Everyday guard with co-printed jaws

The [before-and-after loop](media/teeth-guard.webp) pairs a 95° right-side get-up with a forward roll launched on control slot 17 (0.34 s after the start). The left pane uses the [earlier rounded guard](../rounded-protective-cover.stl). The right uses the [revised cover](rounded-protective-cover.stl). Each uses the same V1.1 receiver, both rail cartridges, stock policies and level floor.

Both ducks carry the upper and lower four-tooth-per-cheek co-prints, the concealed face hardware and the closed magnetic chin hatch. The wearable beard and mask are off. The head adds an estimated 0.55 g over stock. The 16 exposed teeth have separate rigid contact hulls with the jaw held neutral. Flexible tooth deflection, as-printed surface friction and impact protection need physical tests.

The revised cover extends the rounded pad another **0.92 mm on each side**, taking the total width from 94 to 95.84 mm and adding an estimated 0.23 g. It keeps the shield over the board and the 76 mm rail slide. The CAD checks found the vents, latch access and service openings clear.

| Level-floor policy trial | Earlier guard + teeth | Revised guard + teeth |
|---|---:|---:|
| 95° right-side release | Stayed down for 20 s | Resumed walking at 3.58 s, advanced 2.01 m after recovery |
| Roll launched at 0.34 s | 361.8°, kept walking | 361.9°, kept walking |
| Five release angles in four directions | 17/20 | 20/20 |
| Eleven roll launch times | 8/11 | 11/11 |
| Upright walking trial | Pass | Pass |
| Nearby right releases and roll launches | 5/8 | 7/8 |

The 90° right-side start took longer with the revision, increasing from 3.62 to 4.88 s. Both ducks advanced about 2.46 m in the 20 s upright walking trial.

The nearby right-side releases were 93°, 94°, 96° and 97°. The revised duck missed at 97°. Both guards passed launches at 0.30, 0.32, 0.36 and 0.38 s. These deterministic cases helped select the shape. Each count is a tally of the listed simulated starting conditions.

## Reproduce the comparison

Keep the hardware and runtime forks side by side. Start in this directory. Use the project's CAD environment (NumPy, SciPy and manifold3d) for geometry and the runtime qualification environment (MuJoCo and ONNX Runtime) for policy runs. The replay requires both sets of dependencies and Pillow.

1. Generate the head profile with `python head.py` and the cover with `python export-guard.py`. The cover generator reads the six dimensions in [fit.json](fit.json) and checks the rail slide, latch and service openings.
2. Run `python head.py --check` in the physics environment to verify the compiled mass and contact shapes. Run `python qualification.py` for the [matched 32-case sweep](qualification-results.json) and `python neighbors.py` for the [eight nearby starting conditions](neighbors.json).
3. Run `python replay-teeth.py` to render the [loop](media/teeth-guard.webp) and its [replay record](media/teeth-guard-replay.json). The renderer checks the policy and model inputs and compares the rerun metrics with the saved trials before writing the animation.

The new cover replaces only the removable everyday cover. It uses the same rail side and bed orientation as the earlier print. The compatible jaws and one-tooth material coupon are in the [co-print guide](../../../v5-dino-beak/coprint/README.md).
