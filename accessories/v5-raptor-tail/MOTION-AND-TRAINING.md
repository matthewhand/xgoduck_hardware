# V5 tail motion and forward-running experiment

## Head-sized counterbalance study

The V5 head shell spans **124 × 88 × 57 mm** in the policy's home stance. The moving neck subtree includes 260.9 g. Reducing neck pitch by 25° at the fixed standing pose shifted that subtree 31.88 mm forward and added 0.08160 N·m of forward gravity moment. This is an incremental moment relative to the same Duck standing with its head neutral.

The study retained the existing 20.27 g fixed mount. It modeled a tapered hollow tail with an 80 × 50 mm maximum cross section, a 10° upward neutral axis, and zero to three passive pitch hinges with ±16° travel each. The 1.6 mm wall, 8% core fill, 1.24 g/cm³ plastic and 0.8 g per hinge are design estimates. Each convex band has its estimated mass and an inertia approximated by a solid band. The model includes floor contact, the stock head-to-trunk contact, and a simplified joint root. It does not model a drive cable, a servo, tail-to-leg collisions, or manufacturable joints for these larger shells. Length is root to tip.

| One passive hinge | Added mass incl. mount | Rearward offset | Forward distance in 20 s |
| --- | ---: | ---: | ---: |
| Bare Duck | 0 g | 0% | 2.74 m |
| 124 mm | 67.0 g | 92.4% | 2.16 m |
| 132 mm | 70.0 g | 98.9% | 2.11 m |
| 156 mm | 78.9 g | 119.7% | 1.96 m |
| 187 mm | 90.4 g | 149.1% | 1.45 m |

The rearward offset uses the mean gravity moment of the entire added pack in seconds 5–20 of walking, about the lateral axis through the nominal standing foot sites. The reference axis follows trunk translation and heading. It is a common design reference, distinct from the moving stance foot. The denominator is the static 0.08160 N·m head-lean increment. During walking the controller can reposition the head and torso. The 132 mm single-hinge case moved its head-and-neck centre forward 27.5 mm in the trunk frame between the neutral and leaning commands.

At 132 mm the rigid, one-, two- and three-hinge designs offset 98.2%, 98.9%, 100.0% and 100.6%. Each walked approximately 2.11 m. A separate single-hinge comparison held the entire pack at approximately 70 g across all four lengths. The 124, 132, 156 and 187 mm tails then offset 97.0%, 98.9%, 104.5% and 111.8%. Both tail mass and lever arm matter. The 132 mm single-hinge moment was already 0.08043 N·m in the neutral walk, versus 0.08071 N·m while leaning. A mechanism that changes the counterweight as the head moves would require a linkage or commanded motion.

All 16 candidate configurations failed the 90° right-side get-up at nominal estimated mass. The bare Duck stood and resumed walking in 2.86 s. The 132 mm one- and two-hinge versions also failed at 85° and 95° and at ±20% moving-tail mass. At the higher mass both missed the forward walking distance gate. For the nominal 132 mm single hinge, disabling floor contact on just the mount or just the moving tail still failed. Disabling both restored recovery at 2.80 s with the same mass. Reduce floor interference on both surfaces before selecting a print or starting a running-policy training run.

To reproduce, run `python size-study.py RUNTIME size-study.json` and `python render-size-study.py RUNTIME` from this directory in the MuJoCo/ONNX environment. The second command writes the two recorded comparison clips to the runtime assets. [Full results](size-study.json)

## Slender printable tail baseline

The stock walking policy moved the Duck forward with the 25.37 g tail. Both versions failed to get up from the right side. Letting the tail pivot produced almost the same forward speed as locking it. **Revise the fixed mount and repeat the right-side recovery before printing a running prototype.**

| 20-second nominal trial | Bare | Tail locked | Tail passive |
| --- | ---: | ---: | ---: |
| Straight distance at 0.20 m/s command | 2.595 m | 2.443 m | 2.436 m |
| Straight and two turn gates passed | 3/3 | 3/3 | 3/3 |
| Front, back, left, right get-ups passed | 4/4 | 3/4 | 3/4 |
| Right-side return to walk | 2.86 s | Failed | Failed |
| Mean speed at 0.45 m/s command | 0.264 m/s | 0.178 m/s | 0.182 m/s |
| Unsupported phases ≥20 ms between opposite feet at that command | 0 | 0 | 0 |

Each cell is a deterministic simulated case on the same flat floor. The walking and get-up ONNX files, command, joints, initial states and 5 ms physics steps are shared across the three loadouts. Locked and passive versions have identical neutral geometry, component masses, centres of mass and inertia tensors. The moving version has two unactuated hinges with nominal ±16° travel. Only their gravity, contact, viscous drag and sliding friction drive them. The PDF or STL cannot specify friction in a printed pin. The coefficients in the profile are estimates. [Per-case results, gate outputs and model hashes](stock-policy-screen.json) · [Mass layout and CAD frames](tail-layout.json)

The right-side failure also occurred with the short link. With the same mass and only the fixed mount allowed to contact the floor, both lengths still failed. Disabling all attachment-to-floor contact restored recovery at 2.84 seconds for both lengths. That identifies the fixed assembly’s contact envelope as a candidate for redesign. Inspect clearance around the cover and root yoke while the stock body rolls and braces. The collision approximation splits each component into convex pieces. Check on the detailed triangle geometry before changing a particular surface. [Right-side contact ablation](right-side-recovery.json)

## Reproduce the comparison

From this directory, use the existing hardware CAD Python environment for the first two commands, then the runtime’s MuJoCo/ONNX environment for the last two. `RUNTIME` is a local checkout of the matching Duck Lab runtime fork. The standard CAD physics profile is included. The optional second command builds the shorter design for the recovery comparison.

```sh
python tail_physics.py tail-physics.json
python tail_physics.py short-physics.json --length short
python check-tail.py RUNTIME stock-policy-screen.json tail-motion.json
python diagnose-recovery.py RUNTIME short-physics.json right-side-recovery.json
```

The CAD step integrates each actual solid as signed tetrahedra, uses the nominal mass of that individual part and combines its central inertia with the other parts of its link. The simulated hinges have zero armature and zero spring stiffness. The same stock walking and recovery policy drives all 14 actuators. A control slot is 20 ms. Ground-contact measurements run every 5 ms. The four-second browser sample starts two seconds into the straight walk and contains the measured root pose, 14 servo angles and the two measured hinge angles at 25 Hz.

## Prepare a forward-running policy

The companion task starts from Luwu's open-source PPO environment. Its robot loader adds the same two passive hinges and mass layout to Luwu's **backlash training model**. The stock action and joint-observation selectors skip passive joints. The policy still issues 14 motor commands. Its forward target is 0.35–0.55 m/s. In addition to the upstream velocity-tracking and stability rewards, the custom term rewards short unsupported intervals while moving forward upright. A whole-body floor sensor prevents credit for sliding along on the tail. [Upstream training project at the pinned revision](https://github.com/LuwuDynamics/xgoduck_rl/tree/a4c78aacd5eaa724cf246fe12d24bfe1e837fa50) · [Luwu training and export guide](https://wiki.xgorobot.com/kb/the-xgo-series-robot/duck-rl?model=duck)

On a host with the pinned upstream `xgoduck_rl` environment already set up, run these commands **from that training checkout**. Replace `/path/to/tail` with the absolute path to this accessory directory. First run the configuration and model check, then a small smoke training job. Increase the environment count only after checking the GPU memory on that host.

```sh
uv run python /path/to/tail/train-forward.py --check
uv run python /path/to/tail/train-forward.py --env.scene.num-envs 32 --agent.max-iterations 1
```

Local checks compiled the pinned upstream backlash XML with this profile to 37 generalized coordinates, 14 actuators and two additional unpowered hinges. A six-case tensor test rewarded only the upright, forward, fully unsupported example. The full mjlab package and an accessible CUDA device were unavailable on the development machine, so the configuration launch, PPO training and ONNX export still need a GPU-host run. There is no trained tail running checkpoint.

For a candidate exported with Luwu's runtime metadata, repeat the complete bare, locked and passive screen.

```sh
python check-tail.py RUNTIME candidate-report.json candidate-motion.json --walk-policy candidate.onnx
```

The running gate in this experiment requires an upright, fall-free 20-second run averaging at least 0.35 m/s, with eight or more unsupported intervals of at least 20 ms between opposite-foot contacts. Evaluate additional start poses and friction values when a candidate clears this first screen. Recovery from all four sides remains a separate gate. Bench measurements of the printed mass, friction and resting angle are needed to relate the simulated tail to the assembled Duck.
