# Firmware handover: motor endpoints + goto

Hand this to the OH1 **firmware** agent. The Hand Viewer GUI will own **endpoint discovery** (jog + plateau detect). Firmware owns **storage** and **motion primitives**.

Related GUI docs: `hand_viewer/ESP32_COMMANDS.md`, `references/GUI_HANDOVER.md`.

---

## Split of responsibility

| Layer | Owns |
|-------|------|
| **GUI (host)** | Finding candidate min/max (jog, watch `values angles`, mark ends), host-driven demo sweeps |
| **Firmware** | Store min/max per motor, execute `fwd`/`rev`/`goto`/`idle`/`hold`, stream angles |

Do **not** implement firmware auto-find, stall/torque streams, or NVM persistence in this pass.

---

## Must keep working / harden

These commands already exist; keep them reliable (USB Serial + BLE, `\n`-terminated):

| Command | Semantics |
|---------|-----------|
| `min` / `max` | Mark **current pose** as min/max for **all** motors |
| `min N` / `max N` | Same for device motor **N** only |
| `fwd N [seconds]` | Velocity jog forward (default ~2 s) |
| `rev N [seconds]` | Velocity jog reverse |
| `idle` / `idle N` | Stop / coast |
| `hold N` | Hold current angle |
| **`goto N <0..1>`** | Move to calibrated fraction of travel |

### `goto` (critical)

- Requires **both** min and max set for motor N.
- `0` = min end, `1` = max end (linear in encoder space between stored ends).
- Used by GUI Host Demo, threshold/choreography, and margin inset after finding ends.
- Soft endstops on velocity jogs (~5% / ~95%) are fine; do not block `goto` to commanded fraction.

### Telemetry while jogging

- `values angles …` must keep streaming (~10 Hz) during `fwd`/`rev` so the host can detect plateaus (`|Δangle|` small for a hold time).
- Missing / unset → literal `nan`.

### Soft endstops

Keep the ~5% / ~95% jog clamp for calibrated motors — it is good protection.
Just note that it makes the stored range a one-way ratchet from the GUI's point
of view, which is why `cal clear` below is required. The GUI no longer re-marks
`min`/`max` at inset poses (that shrank the range on every run); it applies the
safety margin host-side instead and leaves the device range as discovered.

Calibration remains **RAM-only** until a later persistence task — note that in `help` / comments if useful.

---

## Please add (small)

### `cal clear` / `cal clear N` — needed

Forget the stored min/max for all motors (or motor N) and return them to the
uncalibrated state, exactly as a power cycle does today.

**Why this is needed:** once both ends are set, the soft endstops clamp velocity
jogs to roughly 5%–95% of the stored range. A jog can therefore never travel
back outside the current range, so the GUI cannot re-run endpoint discovery to
*widen* it — a `fwd` looks like it was accepted but the motor does not move.
The only workaround today is to power cycle the glove, which is disruptive mid
experiment.

After `cal clear N`:

- `values motors` reports `nan` for N again
- `fwd` / `rev` jog the full mechanical travel with no soft endstop
- `goto N p` is rejected (or ignored) until both ends are re-marked

### `get minmax`

Reply with stored endpoints in **encoder degrees** so the GUI can sync after reconnect.

**Preferred reply format** (pick one and stick to it; document in `help`):

```
minmax <m1_min>,<m1_max>,<m2_min>,<m2_max>,<m3_min>,<m3_max>,<m4_min>,<m4_max>,<m5_min>,<m5_max>
```

- Ten floats, device order **M1..M5**.
- Use `nan` for an unset end.
- Also acceptable if it fits existing streams better:

```
values minmax <same 10 floats>,ts,<millis>,
```

Emit on Serial and BLE notify (same as `version:`).

---

## Device motor indices (commands / streams)

GUI remaps labels; **firmware command indices stay device order**:

| Device motor | Finger |
|--------------|--------|
| M1 | Pinky |
| M2 | Ring |
| M3 | Middle |
| M4 | Index |
| M5 | Thumb |

`values motors` / `values angles` field order = M1..M5 (Pinky→Thumb).

---

## Out of scope (this pass)

- Firmware auto endpoint-find / `demo`-based discovery
- Current, torque, or stall telemetry for finding ends
- Flash/NVM persistence of min/max
- Changing finger order on the wire

Device `demo` (goto min then max per calibrated motor) may remain as a smoke test; the GUI Host Demo does not depend on it.

---

## Acceptance checks (Serial)

1. `start` → see `values angles …` updating.
2. Jog `fwd 5` / `rev 5` → angles for M5 change; stream does not pause.
3. At open pose: `min 5`. At closed pose: `max 5`.
4. `goto 5 0` then `goto 5 1` → motor reaches both ends.
5. `get minmax` → line with ten values; M5 min/max are finite, others may be `nan`.
6. `cal clear 5` → `values motors` shows `nan` for M5, and `fwd 5` again jogs past
   the previously stored end.
7. `cal clear` → all five motors uncalibrated.
8. Power cycle → ends cleared (`nan`) until re-marked (document RAM-only).
