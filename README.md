# Mechanical Engineering Project Group ไอเอิท

## Delta Haptic Device

- **Team:** 6730156021 Natchakrit Thongraya, 6730317821 Piyathon Banjong, 6730154821 Natthabhat Nilsuwan
- **Purpose:** a low-cost 3-DOF haptic interface. The user holds a handle that moves in 3D; the device can read its
  position and push back with a force. It is a delta (parallel) mechanism with three NEMA17 motors, each driving its
  arm through a rope capstan, built from 3D-printed PETG parts.
- **Status:** unit assembled; CAD final; Simscape Multibody model, symbolic kinematics/dynamics and circle-motion
  animation done.

<img src="media/final_unit_3.jpg" alt="The final unit" height="340"> <a href="media/final_unit_demo.mp4"><img src="media/final_unit_demo_poster.jpg" alt="Video: moving the unit by hand" height="340"></a> <img src="media/delta_symbolic_anim_frame.png" alt="Symbolic circle-motion animation" height="340">

Left: the final unit. Middle: [video](media/final_unit_demo.mp4) of the handle moved by hand (18.5 s). Right: the
symbolic animation moving the hand round a circle.

**Contents:** [1. How the delta works](#1-how-the-delta-works) ·
[2. DOF analysis](#2-dof-analysis) · [3. Circle motion](#3-circle-motion) ·
[4. Running the simulation](#4-running-the-simulation) · [5. Hardware](#5-hardware) ·
[6. Assumptions](#6-assumptions) · [7. Detailed walk-through](#7-detailed-walk-through) · [8. Media](#8-media)

---

## 1. How the delta works

### 1.1 The mechanism

![Kinematic diagram: base, 3 revolute shoulders, 3 upper arms, 6 rods with spherical joints, hand](docs/symbolic-math/img/kinematic_diagram.svg)

Three identical legs stand on the base, 120° apart (at 180°, 300° and 60°). Each leg is a chain:

**motor → rope capstan → upper arm (turns about the shoulder) → two parallel rods → hand**

- The **motor** turns a small drum (rope groove r = 9 mm). A rope wrapped round it drives the **big drum** (r = 76 mm),
  which is part of the upper arm. Ratio **N = 76/9 ≈ 8.44**: the arm turns 8.44× slower than the motor with 8.44×
  the torque, in the opposite direction, with no gears and no backlash. The rope wrap limits the arm to
  **−85.0° … −2.8°** (arm up).
- The **upper arm** (La = 72.1 mm) can only rotate about its shoulder axis (two 606ZZ bearings).
- The **two forearm rods** (Lb = 170 mm, 47 mm apart) have an M6 rod end (a ball joint) at each end. Being equal and
  parallel, they form a **parallelogram**, so the hand edge always stays parallel to the elbow bar.
- The **hand** joins all three legs. Because each leg's parallelogram keeps the hand parallel to a different
  direction, the hand **cannot rotate**: it only translates in x, y, z. Three motor angles give one hand position.

### 1.2 One leg in numbers

![Side view of one leg with points A, B, C, p and O](docs/symbolic-math/img/leg_side_view.svg)

Each leg is written in its own frame (x out along the leg, z up):

| Point | Expression | Meaning |
|---|---|---|
| Aᵢ | [R, T, H] | shoulder pivot, fixed |
| Bᵢ | [R + La·cos qᵢ, T, H − La·sin qᵢ] | elbow, turns with the shoulder angle qᵢ (q > 0 = arm down) |
| Cᵢ | Rzᵀ·p + [r, 0, 0] | where the rods meet the hand; p = (x, y, z) is the hand centre |

| Symbol | Value | Meaning |
|---|---|---|
| R | 80.95 / 79.55 / 78.83 mm | base centre → shoulder, per leg (from the base screw holes) |
| H | 124.1 mm | shoulder height above the base top |
| T | < 0.5 mm | small sideways offset along the shoulder axis |
| La | 72.1 mm | upper arm, shoulder → elbow |
| Lb | 170 mm | forearm rod, eye to eye |
| r | 34 mm | hand centre → hand rod-end bolt line |
| w | 47 mm | spacing of the two rods |
| N | 76/9 ≈ 8.44 | capstan ratio |

### 1.3 The one equation: rods cannot stretch

The parallelogram behaves like a single rod from Bᵢ to Cᵢ, so each leg has exactly one constraint:

$$|C_i - B_i| = L_b$$

With d = Cᵢ − Aᵢ (shoulder → hand attachment) this expands to

$$a\cos q_i + b\sin q_i + c = 0,\qquad a = -2L_a d_x,\quad b = 2L_a d_z,\quad c = |d|^2 + L_a^2 - L_b^2$$

![The elbow lies where the arm circle and the forearm circle cross](docs/symbolic-math/img/forearm_two_circles.svg)

Geometrically the elbow must lie on a circle of radius La round the shoulder **and** on a circle of radius Lb round
the hand attachment. With the half-angle substitution u = tan(q/2) the equation becomes a quadratic, giving the
**inverse kinematics** (hand position → arm angles):

$$q_i = 2\arctan\!\left(\frac{b \pm \sqrt{a^2+b^2-c^2}}{a-c}\right)$$

Two solutions (elbow out, used; elbow folded in, impossible here). When a² + b² − c² < 0 the circles do not meet: the
hand is out of reach, which is the edge of the workspace.

### 1.4 Speed and force: the Jacobian

Differentiating the constraint gives the hand–arm speed relation, the **Jacobian** J:

$$\Phi_p\,\dot p + \Phi_q\,\dot q = 0 \quad\Rightarrow\quad \dot p = J\,\dot q,\qquad J = -\Phi_p^{-1}\Phi_q$$

Physically: a rod cannot stretch, so both of its ends move at the same speed along it. At the home pose (all arms
40° up) the rods are almost exactly at 90° to the arms (best transmission): 1° on every arm lifts the hand 1.57 mm,
1° at a motor moves it about 0.19 mm.

The same J maps forces backwards: a hand force F needs shoulder torques τ = Jᵀ·F, and motor torques
τ_m = −τ/N. Holding the whole device still against gravity takes only **≈ 10.7 mN·m per motor**.

---

## 2. DOF analysis

### 2.1 Links

| # | Link | Count | Moves? |
|---|---|---|---|
| 1 | base: plate, brackets, motors (fixed together) | 1 | no, ground |
| 2–4 | upper arm: big drum, connector, elbow bar | 3 | yes |
| 5–10 | forearm rod: tube + two rod-end housings | 6 | yes |
| 11 | hand | 1 | yes |
| | **total n** | **11** | |

### 2.2 Joints

| Joint | Leg | Connects (link ↔ link) | Physical part | Type | f (DOF) | Removes (6 − f) |
|---|---|---|---|---|---|---|
| J1 | 1 | base (1) ↔ upper arm (2) | shoulder, 2 × 606ZZ | R revolute | 1 | 5 |
| J2 | 2 | base (1) ↔ upper arm (3) | shoulder, 2 × 606ZZ | R revolute | 1 | 5 |
| J3 | 3 | base (1) ↔ upper arm (4) | shoulder, 2 × 606ZZ | R revolute | 1 | 5 |
| J4 | 1 | upper arm (2) ↔ rod a (5) | elbow rod end | S spherical | 3 | 3 |
| J5 | 1 | upper arm (2) ↔ rod b (6) | elbow rod end | S spherical | 3 | 3 |
| J6 | 2 | upper arm (3) ↔ rod a (7) | elbow rod end | S spherical | 3 | 3 |
| J7 | 2 | upper arm (3) ↔ rod b (8) | elbow rod end | S spherical | 3 | 3 |
| J8 | 3 | upper arm (4) ↔ rod a (9) | elbow rod end | S spherical | 3 | 3 |
| J9 | 3 | upper arm (4) ↔ rod b (10) | elbow rod end | S spherical | 3 | 3 |
| J10 | 1 | rod a (5) ↔ hand (11) | hand rod end | S spherical | 3 | 3 |
| J11 | 1 | rod b (6) ↔ hand (11) | hand rod end | S spherical | 3 | 3 |
| J12 | 2 | rod a (7) ↔ hand (11) | hand rod end | S spherical | 3 | 3 |
| J13 | 2 | rod b (8) ↔ hand (11) | hand rod end | S spherical | 3 | 3 |
| J14 | 3 | rod a (9) ↔ hand (11) | hand rod end | S spherical | 3 | 3 |
| J15 | 3 | rod b (10) ↔ hand (11) | hand rod end | S spherical | 3 | 3 |
| **Total** | | | | **j = 15** | **Σf = 39** | **Σ = 51** |

R revolute: 1 rotation about the shoulder axis. S spherical (rod end): 3 rotations about the ball centre (within
±13° tilt), no translation.

### 2.3 Grübler–Kutzbach (spatial)

$$M = 6(n - 1 - j) + \sum_{i=1}^{j} f_i = 6(11 - 1 - 15) + 39 = -30 + 39 = \mathbf{9}$$

$$\sum f_i = \underbrace{1+1+1}_{J1\text{–}J3} + \underbrace{3\times 6}_{J4\text{–}J9} + \underbrace{3\times 6}_{J10\text{–}J15} = 39$$

| | Count | Meaning |
|---|---|---|
| idle DOF | 6 | each rod can spin about its own axis between its two balls; nothing else moves |
| useful DOF | 9 − 6 = **3** | hand translation x, y, z (orientation locked by the three parallelograms) |

**Drive chain:** each motor rotor adds a body and a revolute joint (+1) but its rope ties it to the arm (−1): no
change. **3 motors drive the 3 useful DOF**, so the device is fully actuated and not redundant.

**Simscape model:** it uses a universal joint (f = 2) at the elbows J4–J9, which removes the rod spins:
M = 6(11 − 1 − 15) + 3·1 + 6·2 + 6·3 = **3**.

---

## 3. Circle motion

### 3.1 Why the circle is about 30 mm

The arms alone could carry the hand round a circle of about 80 mm radius. The limit is the **rod ends**: an M6 rod
end tilts at most ≈ 13° out of its plane. Moving the hand sideways by p·vᵢ (vᵢ = shoulder-axis direction) tilts leg
i's rods by

$$\text{tilt}_i = \arcsin\frac{|p\cdot v_i - T_i|}{L_b}$$

which reaches 13° at a radius of about 37 mm. Both circle motions therefore use the largest circle that keeps every
arm inside its rope end-stops **and** every rod end below 13° (local functions `maxRadius` and `rodEndTilt`), with a small margin.

### 3.2 Symbolic animation (`delta_symbolic_anim.mlx` / `.m`)

Easiest to read as the **live script** [`simulation/delta_symbolic_anim.mlx`](simulation/delta_symbolic_anim.mlx): open it in MATLAB and the explanations, equations, results, plots and animation appear next to the code (saved with its outputs, so they show even before running). The same code is in [`delta_symbolic_anim.m`](simulation/delta_symbolic_anim.m), readable on GitHub.

The hand follows a horizontal circle, 2 laps of 5 s:

$$p(t) = \big[x_0 + R_c\cos\tfrac{2\pi t}{5},\ \ y_0 + R_c\sin\tfrac{2\pi t}{5},\ \ z_0\big],\qquad R_c = 30.7\text{ mm},\ z_0 = 313\text{ mm}$$

1. **Symbols and points** Aᵢ, Bᵢ(qᵢ), Cᵢ(p) (section 1.2).
2. **Solve** |Cᵢ − Bᵢ| = Lb for qᵢ(x, y, z) with `solve()` and u = tan(q/2) (section 1.3), keeping elbow out.
3. **Substitute** p(t): `q_t = subs(qIK, p, pt)`, then `diff(q_t, t)` and `diff(q_t, t, 2)` give arm speed and
   acceleration (the chain rule builds J automatically).
4. **Check** q(t) against the numeric inverse kinematics `deltaIK` (to 1e-9 rad), the rope end-stops and the
   rod-end tilt; the script stops with a message if any check fails.
5. **Animate** with `fanimator` / `playAnimation`: upper arms (red), rods (green), hand (blue), the circle path
   (magenta, centre marked +) and a magenta dot riding on the hand.

Result: arms between −59.5° and −31.2°, rod-end tilt at most 10.8°.

<img src="media/delta_symbolic_anim_q.png" alt="Shoulder angle, speed and acceleration along the circle" width="640">

Each arm swings once per lap; the three legs are 120° out of phase, like the three phases of a motor.

### 3.3 Simscape Multibody circle (`circle_demo.m`)

The full multibody model (all CAD parts, weighed masses, rope capstans, end-stops) is driven by "grabbing" the top
of the handle: its local function `run_grab_path` adds a 6-DOF joint whose x, y, z follow the path, so the motors are passive and the
arms, rods and drums follow as in the real unit when you move the handle by hand. The circle has a 3° margin from
the rope stops: **radius 28.3 mm, 3 laps, handle tracking error 0.07 mm, rod-end tilt ≤ 10°.**

### 3.4 Real-life use

**Where delta haptic devices are used.** The delta layout is the classic choice for desktop haptic devices: the
motors sit still on the base, so the moving parts are light, the three legs share the load, and the hand stays level
without any wrist joints. Well-known examples are the Novint Falcon (consumer games) and the Force Dimension omega /
delta devices (research and medical). Typical uses:

| Use | What the device does |
|---|---|
| Surgical and medical training | the trainee moves the handle like a tool; the device pushes back to make virtual tissue feel soft or bones feel hard |
| Teleoperation | the handle steers a distant robot (surgical, underwater, nuclear); forces measured at the robot are played back to the hand |
| Rehabilitation | patients practise guided arm movements; the device assists, resists, or measures how smoothly they move |
| Virtual prototyping and CAD | designers "touch" a 3D model to feel shapes, clearances and assembly fits before anything is made |
| Research and education | testing control, perception and force rendering; teaching kinematics on a real parallel robot |
| Games and simulators | feeling recoil, terrain, steering or impacts through the handle |

**Why a circle path.** A horizontal circle is one of the most common test and exercise paths for a 3-DOF device:

| Real-life use of the circle | Why the circle helps |
|---|---|
| **Calibration and accuracy test** | going round a circle uses all three motors in turn (each arm swings once per lap, 120° apart, see the plot in 3.2), so any error in one leg, a wrong length or an encoder offset shows up as a distorted or shifted circle |
| **Tracking / control test** | a circle at constant speed never stops, so it shows lag, overshoot and friction (for example stepper detent torque) better than point-to-point moves |
| **Workspace check** | the largest circle that fits shows the usable workspace; here it shows the rod-end tilt limit (≈ 30 mm radius), not the arm travel |
| **Rehabilitation exercise** | tracing circles is a standard upper-limb therapy task; the device can guide the hand, resist it, or score how round and smooth the circle is |
| **Haptic guide ("virtual fixture")** | the device renders a circular groove or wall: the user moves freely along the circle and feels a spring force pulling back when leaving it, as used to guide a surgical tool or teach a motion |
| **Demonstration** | it is the simplest motion that makes all three legs work together, which is why both simulations here use it |

**What this unit can do on a circle.** Following the analysis above: a circle of about 28–31 mm radius at a handle
height of about 305–340 mm, kept inside the rope end-stops and the 13° rod-end tilt; holding the device against
gravity needs only ≈ 10.7 mN·m per motor, and 1 N at the hand needs roughly 3.6–9.5 mN·m at a motor. With the
current stepper drivers it can be moved round the circle by hand or driven as a position robot; rendering forces
(guides, walls, assistance) needs torque-controlled drives and encoders (see Assumptions 7–8).

---

## 4. Running the simulation

MATLAB R2026a with Simulink, Simscape, Simscape Multibody and Symbolic Math Toolbox.

**Quickest:** download the whole repository, open [`simulation/delta_haptic.slx`](simulation/delta_haptic.slx) and press **Run**. The model finds the STEP files in `hardware/mechanical/cad/` by itself when it opens (it stores only the file names), so it works from any folder as long as the repository layout is kept. Requires R2026a or newer.

Or open MATLAB in `simulation/` and use the scripts:

| Command | What it does |
|---|---|
| `build_delta_haptic` | builds the Simscape Multibody model `delta_haptic.slx` from the STEP files; all parameters `P.*` are at the top |
| `circle_demo` | moves the handle round the largest allowed circle, 3 laps, in the 3D view (`circle_demo(false)` without real-time pacing) |
| `delta_symbolic_anim` | symbolic inverse kinematics, speed and acceleration, plots and the circle animation (or open `delta_symbolic_anim.mlx` and press **Run All**) |

| File | Purpose |
|---|---|
| `delta_haptic.slx` | ready-to-run Simscape Multibody model (built by `build_delta_haptic.m`) |
| `build_delta_haptic.m` | model builder and parameter set `P` (geometry, weighed masses, drive, limits, colours); rebuilds `delta_haptic.slx` after changes |
| `circle_demo.m` | circle motion on the multibody model: builds the path, adds the "grab" joint and runs it; helpers (`run_grab_path`, `maxRadius`, `rodEndTilt`, `deltaFK`, `deltaIK`) are local functions at the end of the file |
| `delta_symbolic_anim.mlx` | live script of the symbolic math circle animation: 10 explained sections with equations, saved with outputs (open in MATLAB) |
| `delta_symbolic_anim.m` | same code as a plain script (readable on GitHub); device parameters at the top, numeric kinematics helpers as local functions at the end |

**Model structure:** top level Environment, Leg 1–3, Hand, MotorTorque. Each leg: motor + bracket → drive (motor
shaft, small drum, rope as a no-slip gear constraint) → upper arm (shoulder joint with rope end-stops, connector, big
drum, 2 × 606ZZ, rope anchors) → 2 forearm rods (universal joint at the elbow, ball joint at the hand). Part colours
match the built unit (`col.*` in `build_delta_haptic.m`). STEP files are read from `hardware/mechanical/cad/`
(the model's load step puts that folder on the MATLAB path). Only `delta_haptic.slx` is stored; the circle demo
creates its own copy when it runs.

**Verified during development:** forward kinematics vs Simscape 5e-13 mm; inverse kinematics round trip 1e-15 rad;
Jacobian, velocity and acceleration vs finite differences ≈ 1e-7; gravity compensation in Simscape 0.00 % residual;
free-fall start acceleration symbolic vs Simscape within 0.9 %; bracket vs base screw holes within 0.005 mm.

---

## 5. Hardware

STEP files in [`hardware/mechanical/cad/`](hardware/mechanical/cad/) (read directly by the model; keep the names).
All masses weighed.

| File | Part | Qty | Mass (each) |
|---|---|---|---|
| `Base V4.step` | base plate | 1 | 250 g |
| `Motor Bracket v6.STEP` | motor bracket + shoulder pivot (white PETG) | 3 | 32.2 g |
| `Big Drum V6.STEP` | big capstan drum sector r 76 mm, part of the upper arm (yellow PETG) | 3 | 31.3 g |
| `connector v6.STEP` | upper arm / elbow bar (grey PETG) | 3 | 9.1 g |
| `Small Drum v5.STEP` | small capstan drum on the motor shaft, r 9 mm (blue PETG) | 3 | 4.2 g |
| `Link V.2.step` | forearm tube Ø10 × 100 (mint PETG) | 6 | 6.8 g |
| `hand V.3.step` | hand with handle (blue PETG) | 1 | 12.9 g |
| `Nema17 42BYGH40.step` | NEMA17 stepper motor | 3 | 330 g |
| `M6 Rod End L51.step` (+ ball / housing) | M6 rod end, 51 mm stud | 12 | 20 g |
| `606ZZ.step` | bearing, 2 per big drum | 6 | 5.7 g |
| `M6x12 ISO4762.step` | rod-end screws | 12 | 6.3 g |
| `M3x6 ISO4762.step` | motor, bracket and drum-stub screws | 27 | 0.66 g |
| `M3x10 ISO4762.step` + M3 nut (ISO 4032) | rope clamp in the big drum slot; the nut is drawn in the model as a built-in hexagon (5.5 mm across flats, 2.4 mm), no STEP file | 6 | 1.51 g per pair |

Electronics (bench test): 3 × TB6600 stepper drivers, ESP32-S3 controller.

---

## 6. Assumptions

1. **Rigid parts, ideal joints:** no flex in the printed parts, no play in bearings or rod ends.
2. **Parallel rods** stay parallel and equal, so the hand never tilts.
3. **Rope** does not slip or stretch (ratio exactly 76/9); end-stops modelled as stiff springs (2000 N·m/rad,
   2 N·m·s/rad).
4. **Rod-end tilt limit 13°** (typical M6 rod end); checked along the paths, not enforced by the joints.
5. **Masses:** every part uses its weighed mass spread over its CAD shape; rod-end balls at steel density; the
   M3×10 + nut pair split by the nut's ISO size; motor rotor 54 g·cm² (typical NEMA17).
6. **Friction:** only viscous shoulder damping 2 mN·m·s/rad (estimate); no bearing/rope friction, detent torque or
   backlash.
7. **Motors** are ideal torque sources (the TB6600 step/direction drivers cannot command torque; real force
   feedback needs torque-controlled drives and encoders).
8. **Sensing:** joint angles known exactly; no encoder resolution or control delay.
9. **Dynamics:** each arm one rigid body plus N² × rotor inertia; each rod's mass split half to elbow, half to hand.
10. **Gravity** 9.81 m/s² along −z; base level and fixed; collisions not modelled.

---

## 7. Detailed walk-through

Step-by-step derivations with diagrams and worked numbers, in [`docs/symbolic-math/`](docs/symbolic-math/):

| Part | Topic |
|---|---|
| [1](docs/symbolic-math/01-leg-points.md) | the points of one leg (side and top view) |
| [2](docs/symbolic-math/02-forearm-equation.md) | the forearm-length equation and the two-circle picture |
| [3](docs/symbolic-math/03-solving-for-q.md) | solving for q with u = tan(q/2) |
| [4](docs/symbolic-math/04-velocity-jacobian.md) | velocity and the Jacobian |
| [5](docs/symbolic-math/05-acceleration.md) | acceleration |
| [6](docs/symbolic-math/06-dynamics.md) | dynamics: motor torque to hand force |
| [7](docs/symbolic-math/07-animation-script.md) | how the circle animation script works |
| [8](docs/symbolic-math/08-dof-analysis.md) | degrees of freedom and kinematic diagram |

Scripts named there but not in this repository (`delta_symbolic.m`, `delta_kinematics*.m`, `delta_dynamics.m`,
`delta_motor_torque.m`) were used to derive and verify the results.

---

## 8. Media

**Final unit**

<img src="media/final_unit_1.jpg" width="200"> <img src="media/final_unit_2.jpg" width="200"> <img src="media/final_unit_3.jpg" width="200"> <img src="media/final_unit_4.jpg" width="200">
<img src="media/final_unit_5.jpg" width="200"> <img src="media/final_unit_6.jpg" width="200"> <img src="media/final_unit_7.jpg" width="200"> <img src="media/final_unit_8.jpg" width="200">

**Video:** [final_unit_demo.mp4](media/final_unit_demo.mp4)

**First prototype**

<img src="media/prototype_1.jpg" width="200"> <img src="media/prototype_2.jpg" width="200"> <img src="media/prototype_3.jpg" width="200">

---

## Repository structure

| Path | Contents |
|---|---|
| `simulation/` | ready-to-run Simscape Multibody model (`delta_haptic.slx`), its builder, circle demo, symbolic circle animation |
| `hardware/mechanical/cad/` | final STEP files |
| `hardware/bom.csv` | bill of materials (template) |
| `docs/symbolic-math/` | detailed walk-through, parts 1–8, and diagrams |
| `media/` | photos and video of the unit, animation images |
