# Simulation

Multibody model, symbolic kinematics/dynamics and demos of the 3-DOF capstan-drive delta haptic device.

**Software:** MATLAB R2026a with Simulink, Simscape, Simscape Multibody and Symbolic Math Toolbox.
**CAD input:** STEP files in [`../hardware/mechanical/cad/`](../hardware/mechanical/cad/). `build_delta_haptic`
finds them automatically: it uses the first of `Model/` (next to the scripts), `../hardware/mechanical/cad/`, or the
script folder itself that contains `Base V4.step`. The `.slx` models store full STEP paths, so after moving the CAD
files run `build_delta_haptic` (and the demo you want) again.

**Colours** match the built prototype (photos 2026-11-11) and are set in one table, `col.*`, near the top of
`build_delta_haptic.m`: yellow big drums, white brackets, blue small drums and hand, grey elbow connectors, mint
forearm tubes, wooden base, silver steel parts, red rope.

## Quick start

Open MATLAB in this folder, then:

| Command | What it does |
|---|---|
| `build_delta_haptic` | Builds `delta_haptic.slx` (Simscape Multibody) from the STEP files; all parameters `P.*` are at the top |
| `circle_demo` | Grabs the handle top and moves it round the largest allowed circle, 3 laps (live 3D view) |
| `helix_demo` | Traces the edge of the workspace: lowest and highest point in each of 3 laps, 7 s per lap |
| `delta_symbolic_anim` | Classroom-style symbolic IK (`solve`), velocity/acceleration (`diff`) and a `fanimator` animation |

`circle_demo(false)` / `helix_demo(false)` run without real-time pacing. Generated models (`*.slx`) are not
stored in the repo; build them with the commands above.

## Files

| File | Purpose |
|---|---|
| `build_delta_haptic.m` | Model builder and parameter set `P` (geometry, masses, drive, limits) |
| `circle_demo.m`, `helix_demo.m` | Demo paths |
| `run_grab_path.m` | Shared: copies the model, adds a 6-DOF "grab" joint on the handle top and drives it along a path |
| `maxRadius.m` | Largest circle at a height that stays inside the rope end-stops and the rod-end tilt limit |
| `rodEndTilt.m` | Rod-end misalignment angle per leg for a hand position |
| `deltaFK.m`, `deltaIK.m` | Numeric forward / inverse position kinematics |
| `delta_symbolic.mat` | Device parameters `P` used by the symbolic animation |
| `delta_symbolic_anim.m` | Symbolic IK + animation (every variable labelled in the code) |

## Model

- **Top level:** Environment, Leg 1-3 (at 180 / 300 / 60 deg around the base), Hand, MotorTorque.
- **Leg:** motor + bracket → drive (motor shaft, small drum, rope as a no-slip gear constraint, ratio 76/9) →
  upper arm (shoulder joint with rope end-stops, connector, big drum, 2× 606ZZ, rope anchors) → 2 forearm rods
  (universal joint at the elbow, ball joint at the hand; rod-end ball fixed to its screw, housing on the rod).
- **Bodies:** STEP solids; printed parts scaled to their weighed masses (PETG 1.27 g/cm³), steel at 7850 kg/m³.

### Key parameters (in `build_delta_haptic.m`)

| Symbol | Value | Meaning |
|---|---|---|
| `R` | 80.95 / 79.55 / 78.83 mm | base centre → shoulder pivot, per leg (from the base screw holes) |
| `H` | 124.1 mm | shoulder pivot height above the base top |
| `La` | 72.1 mm | upper arm, pivot → elbow |
| `Lb` | 170 mm | forearm, rod-end eye → eye (measured) |
| `r` | 34 mm | hand centre → hand rod-end bolt line |
| `N` | 76/9 ≈ 8.44 | capstan ratio (motor and arm turn opposite ways) |
| `qMin`, `qMax` | −85.0°, −2.8° | rope end-stops (q > 0 = arm down) |
| `tiltMax` | 13° | rod-end misalignment limit (typical M6 rod end) |
| `b` | 2e-3 N·m·s/rad | shoulder damping (estimate) |

## Kinematics and dynamics

- Loop closure per leg: Φᵢ = |p + r·uᵢ − Eᵢ(qᵢ)|² − Lb² = 0, elbow Eᵢ = (Rᵢ + La cos qᵢ)uᵢ + tᵢvᵢ + (H − La sin qᵢ)e_z.
- Closed-form IK (arms-up branch), Jacobian J = −Φₚ⁻¹Φ_q, ṗ = J q̇, p̈ = J q̈ − Φₚ⁻¹c.
- Reduced dynamics: (M_q + m_p JᵀJ) q̈ = τ_s − G − b q̇ − Jᵀ(m_p(β + g e_z) − F), motor torque τ_m = −τ_s/N.

## Verified results

Checked during development (the checking and dynamics scripts are not in this repository; see
[the symbolic math walk-through](../docs/symbolic-math/) for the derivations):

| Check | Result |
|---|---|
| Forward kinematics vs Simscape | 5e-13 mm |
| IK round trip | 1e-15 rad |
| Jacobian, velocity, acceleration vs finite differences | ~1e-7 (relative) |
| Gravity compensation in Simscape | 0.00 % residual acceleration, ~10.7 mN·m per motor at q = −40° |
| Free-fall start acceleration, symbolic vs Simscape | within 0.9 % |
| Base screw holes vs bracket | within 0.005 mm (front), 0.013 mm (rear slot) |
| Circle demo (13° rod-end limit) | radius 28.3 mm, tracking 0.07 mm, tilt ≤ 10° |
| Helix demo | handle height 278.8–384.2 mm, widest circle 35.6 mm, tracking 0.28 mm, tilt ≤ 12.5° |

## Assumptions

- All parts rigid; joints have no play.
- Each pair of forearm rods stays parallel, so the hand stays level.
- Rope: no slip, no stretch; end-stops modelled as stiff springs (2000 N·m/rad, 2 N·m·s/rad).
- Masses: all parts weighed (`info.txt`), including hand, bearings, screws and nuts; rod-end balls at steel
  density; motor rotor 54 g·cm² (typical NEMA17).
- Symbolic dynamics: arm = one rigid body about its pivot plus N² × rotor inertia; each rod's mass split half to
  the elbow, half to the hand (exact for gravity, approximate for inertia).
- Not modelled: bearing and rope friction, stepper detent torque, backlash, collisions, motor current/speed limits,
  sensing and control delay.
- **Rod-end tilt limit 13° is assumed**; it is checked along the demo paths but not enforced by the joints.
  Change `P.tiltMax` if the rod-end datasheet gives another value.

## Notes

- `build_delta_haptic` is a script: its variables stay in the workspace (`delta_symbolic` clears them).
- Simscape's default damping unit is per deg/s; the model sets `N*m*s/rad` explicitly.
