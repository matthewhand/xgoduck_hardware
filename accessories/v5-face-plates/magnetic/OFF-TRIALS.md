# Walking, get-up and rolling with the decorative parts removed

The Duck with the concealed head mounts and the bare recessed backpack receiver passed all nine nominal trials. It walked **2.59 m in 20 seconds**, got up from all four sides and completed four rolls. With the everyday protective guard and both rail cartridges fitted, it walked and rolled, but stayed down after the back and right-side starts.

| Stock policies on a flat floor | Stock Duck | Concealed mounts and flush receiver | Same Duck with everyday guard |
|---|---:|---:|---:|
| Total estimated robot mass | 797.79 g | 798.48 g | 815.55 g |
| Forward walk in 20 s | 2.59 m | 2.59 m | 2.51 m |
| Front get-up | 2.92 s | 3.68 s | 3.00 s |
| Back get-up | 2.76 s | 2.78 s | Still down at 20 s |
| Left get-up | 2.56 s | 2.56 s | 2.56 s |
| Right get-up | 2.86 s | 2.86 s | Still down at 20 s |
| Shipped roll at four launch times | 4/4 | 4/4 | 4/4 |
| Completed roll rotation | 361.1–361.4° | 360.9–361.5° | 351.1–361.8° |

The roll policy starts under its own power from standing. The get-up policy starts with the Duck already tipped 90°. These are different maneuvers. All four guarded rolls ended upright and resumed walking with zero recovery transitions. After the guarded back and right starts, the policy never returned to walking during the 20 s trial.

In a diagnostic rerun, the Duck kept the guard's full mass, centre of mass and inertia while its guard and cartridge contact surfaces were disabled. It recovered from the back in **2.78 s** and the right in **2.84 s**. This identifies the modeled contact geometry as the difference in these two failures. The next design test is to change the cover's ground contact profile and repeat those starts with contacts active.

## What stayed installed

The stripped-down head contains four brow steel targets and internal keepers, two chin steel targets and their hatch, and eight captive square nuts across the rigid upper and lower cheek walls. The mask, beard, magnets, tooth strips and screws are removed. The generator integrates the *removed* plastic and the *added* plastic and metal separately in registered CAD coordinates, including each component's centre of mass and inertia. The resulting head is **1.061 g heavier** than stock. The empty recessed rear receiver removes **0.366 g**, giving **+0.695 g** for the whole Duck. Fitting the plain cover and two cartridges adds another **17.064 g**.

The stock head contact mesh represents the closed jaw throughout. Every concealed insert is geometrically inside the stock hull. The cheek screw access openings change the outer convex hull of the upper jaw by at most 0.0015 mm. The bare face and lower jaw retained their stock hull. The guard uses convex sections of the actual cover and cartridges. Filament density, hardware mass, floor friction and impact behavior remain measurements to check on the physical build.

## Reproduce the comparison

From this directory run `python check-off.py profile` in the CAD environment (manifold3d, SciPy, NumPy), then `python check-off.py trials` in the qualification environment (MuJoCo, ONNX Runtime, NumPy). Both accept `--runtime PATH` if the runtime checkout is elsewhere. [Head CAD mass and inertia](off-head-profile.json) · [Per-trial results and input hashes](off-results.json) · [Plain cover and cartridge CAD profile](../../v5-slide-on-backpack/removable-rails/plain-guard.py)

The stock walk and get-up run at 50 Hz with 200 Hz physics and a 0.20 m/s forward command. Passing a fall requires getting up within 8 s, covering at least 0.75 m along the recovered heading, taking five steps on each foot and ending upright. The roll study starts the shipped 1.7 s movement after settling for 0.2, 0.5, 1.0 and 2.0 s and checks the turn, landing and following 2.5 s of walking. These are deterministic starts, not estimates of field reliability.
