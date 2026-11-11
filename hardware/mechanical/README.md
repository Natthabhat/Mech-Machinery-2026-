# Mechanical hardware

Final CAD of the delta haptic device, as STEP files in [`cad/`](cad/). The Simscape model in
[`../../simulation/`](../../simulation/) reads these files directly, so keep the file names unchanged.

## Printed parts (PETG, 1.27 g/cm³)

| File | Part | Qty | Weighed mass |
|---|---|---|---|
| `Base V4.step` | Base plate, 3 bracket pads | 1 | 250 g |
| `Motor Bracket v6.STEP` | Motor bracket (motor + shoulder pivot) | 3 | 32.2 g |
| `Big Drum V6.STEP` | Big capstan drum sector (r 76 mm), part of the upper arm | 3 | 31.3 g |
| `Big Drum Bushing.STEP` | Big drum bushing | | |
| `connector v6.STEP` | Upper arm / rod-end crossbar | 3 | 9.1 g |
| `Small Drum v5.STEP` | Small capstan drum on the motor shaft (rope groove r 9 mm) | 3 | 4.2 g |
| `Shaft End.STEP` | Shaft end | | |
| `Link V.2.step` | Forearm tube Ø10 × 100 | 6 | 6.8 g |
| `hand V.3.step` | Hand (effector) with handle | 1 | not weighed |

## Bought parts

| File | Part | Qty | Mass |
|---|---|---|---|
| `Nema17 42BYGH40.step` | NEMA17 stepper motor 42BYGH40 | 3 | 330 g |
| `M6 Rod End L51.step` | M6 rod end, M6×16 thread (51 mm stud), modelled as `M6 rod end ball.step` + `M6 rod end housing L51.step` | 12 | 20 g |
| `606ZZ.step` | Bearing 606ZZ, 2 per big drum | 6 | |
| `M6x12 ISO4762.step` | Rod-end screws | 12 | |
| `M3x6 ISO4762.step` | Bracket → base screws (4 per leg), drum stub screw (1 per leg) | 15 | |
| `M3x10 ISO4762.step`, `M3 nut ISO4032.step` | Rope clamp in the big drum slot | 6 | |
| `M2.5x6 ISO4762.step` | Motor → bracket screws (NEMA17 is usually M3, check) | 12 | |

Rope: passes through Ø4 holes at both ends of the big drum arc and is clamped by an M3×10 + nut in the drum's
tension slot.

## Fusion 360 scripts

[`fusion-scripts/ISO4762 Screws/`](fusion-scripts/ISO4762%20Screws/) is a Fusion 360 script that models ISO 4762
socket head screws (used for M3×6 and M2.5×6). Run it from Fusion 360 → Utilities → Scripts and Add-Ins.
