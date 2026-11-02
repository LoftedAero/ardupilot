# F35B.lua — SITL Test Plan

Verifies the door subsystem's functionality and edge cases in SITL with Mission Planner (MP)
open on the side. Run with `DOOR_DEBUG = 1` so the script narrates every decision.

## 0. Setup

**Vehicle / frame.** Load your F-35B parameter set (or the closest tiltrotor + dedicated-lift
model) into SITL so the **tilt output** (`DOOR_TILT_FN`, default 41) and the **lift motor**
(`DOOR_LIFT_FN`) are real and move. Everything downstream depends on these two signals, so
Group 1 verifies them first.

```bash
Tools/autotest/sim_vehicle.py -v ArduPlane -f quadplane --console --map
# then in MAVProxy: param load <your_f35b.parm>   (or set params individually)
```

**Script.** `param set SCR_ENABLE 1`, copy `F35B.lua` into the sim `scripts/` dir, reboot.
Set the parameters from the consolidated list, plus:

```
DOOR_ENABLE 1
DOOR_DEBUG  1
```

**Confirm `FLTMODE_CH` ≠ `DOOR_MODE_CH` (8).** Point `FLTMODE_CH` at a spare channel used as
the MANUAL override.

**Driving inputs from MAVProxy:**
- Selector (3-pos on ch 8): `rc 8 1000` (QSTABILIZE), `rc 8 1500` (QLOITER), `rc 8 2000` (FBWA).
- Override switch (on `FLTMODE_CH`): `rc <ch> <pwm>` to the FBWA / MANUAL positions.
- GCS mode command: `mode QHOVER`, `mode FBWA`, etc. Arm/disarm: `arm throttle` / `disarm`.
- Airspeed: fly it, or `param set SIM_WIND_SPD <x>` / throttle to move airspeed across thresholds.

**Watch in MP** — add to the Quick tab: `DOOR_ST` (0 closed / 1 partial / 2 open / 3 opening),
`DOOR_PWM`, `VTOL_DLY`, `TILT_POS`, `LIFT_PWM`. Keep the Messages tab visible.

**Debug messages you'll see (prefix `F35B`, all ≤50 chars):**
- `sel pos N -> mode M (cur C)` — a selector move was acted on
- `inbound M: opening doors X.Xs` — inbound gate started
- `entering VTOL M` — gate finished, VTOL commanded
- `VTOL->VTOL M now` · `FW M now` — instant switches
- `<STATE> m=M v=Y/N t=T l=L as=A` — on every door-position change (t/l are tilt/lift PWM)
- `doors latched FULL` / `full latch reset` — the per-session full latch
- `F35B hb st=S t=T l=L as=A` — ~2 s heartbeat (t/l = tilt/lift PWM)

Production (always on): `doors opening…`, `doors open - entering VTOL`, `doors closed`,
`VTOL entered externally…`, `MANUAL override active - selector ignored`,
`mode changed, VTOL entry cancelled`, `Internal Error: …`.

---

## 1. Telemetry foundation (do this first)

| ID | Steps | Expect / Pass |
|----|-------|---------------|
| 1a | Disarmed, fixed-wing. Read `TILT_POS`. | Near **1000** (nacelles forward). If ~0, `DOOR_TILT_FN` is wrong — fix before continuing. |
| 1b | **Armed / in flight**, enter QHOVER, let nacelles come up. Read `TILT_POS`. | Sweeps across its **PWM** range (1000↔2000) as the nacelles move. Note the forward-flight value → set `DOOR_TILT_FWD` there. (Disarmed on the ground it may not move — test armed.) |
| 1c | With `DOOR_LIFT_FN` set: disarmed vs armed-hover, read `LIFT_PWM`. | Low (stopped) disarmed; higher when the lift motor runs. Set `DOOR_LIFT_OFF` between them. |
| 1d | Watch `DOOR_ST` while idle. | Stable, not flickering. |

---

## 2. Config invariant — no fixed-wing blip

| ID | Steps | Expect / Pass |
|----|-------|---------------|
| 2a | Override switch static. Sweep `rc 8` through all 3 positions. | **Mode does not change on its own** from the sweep (only the script's own commands move it). Proves `FLTMODE_CH` ≠ 8. |

---

## 3. Core manual transitions

| ID | Steps | Expect / Pass |
|----|-------|---------------|
| 3a | **Inbound (fast):** in FBWA at cruise, `rc 8 1500` (QLOITER). | `dbg: inbound … holding fixed-wing`; `DOOR_PWM→1500`; **mode stays FBWA** for `DOOR_ENTRY_MS` (`VTOL_DLY` counts down); then `doors open - entering VTOL`, mode→QLOITER. **No VTOL motor/tilt motion before the mode flip.** As it slows, `DOOR_PWM→2000` and `dbg: latched FULL`. |
| 3b | **Inbound (slow):** fly FBWA below `DOOR_PART_AS`, select QLOITER. | Doors go to **full (2000)** through the entry, not partial. |
| 3c | **Outbound:** hovering QLOITER, `rc 8 2000` (FBWA). | Mode→FBWA immediately (`dbg: fixed-wing target … immediately`); doors stay open, neck full→partial as speed passes `DOOR_PART_AS`; **close only when `TILT_POS` reaches forward (`DOOR_TILT_FWD`) AND `LIFT_PWM`≤`DOOR_LIFT_OFF`** → `doors closed`. |
| 3d | Full round trip FBWA→QLOITER→hover→FBWA→cruise. | Choreography correct both ways; ends `DOOR_ST=0`. |

---

## 4. Airspeed door position, latch-full, chatter

| ID | Steps | Expect / Pass |
|----|-------|---------------|
| 4a | Note airspeed where doors neck 2000→1500 **outbound**. | ≈ `DOOR_PART_AS`. |
| 4b | Note airspeed where doors open 1500→2000 **inbound** (in VTOL, decelerating). | ≈ `DOOR_FULL_AS`. |
| 4c | **Latch-full:** after 3a settles full in QLOITER, push speed back up above `DOOR_FULL_AS`. | `DOOR_ST` **stays 2 (full)** — no neck to partial. |
| 4d | Transition out to FBWA then back into QLOITER fast. | `dbg: session ended - full latch reset`; the fresh fast entry necks to partial again (latch is per-session). |
| 4e | **Chatter:** hold a steady airspeed right at a threshold ~30 s. | `DOOR_PWM` does **not** oscillate; Messages stays quiet (debounce). |

---

## 5. Close-safety gates (both must be satisfied)

| ID | Steps | Expect / Pass |
|----|-------|---------------|
| 5a | During an outbound transition, watch `TILT_POS` vs door close. | Doors do **not** close until `TILT_POS` reaches forward (`DOOR_TILT_FWD`) (waits out the intermediate airspeed-wait angle). |
| 5b | With `DOOR_LIFT_FN` set: after nacelles reach forward, watch `LIFT_PWM`. | Doors stay open until `LIFT_PWM`≤`DOOR_LIFT_OFF`, then close — i.e. not on a still-spinning lift motor. |
| 5c | Temporarily set `DOOR_TILT_FWD` to the non-forward end so nacelles never read "forward". | Doors **never close** (fail-safe open). Restore afterward. |

---

## 6. Selector edge cases

| ID | Steps | Expect / Pass |
|----|-------|---------------|
| 6a | In QLOITER, `rc 8 1000` (QSTABILIZE). | `dbg: VTOL->VTOL … immediately`; instant switch, **no** "opening", no delay. |
| 6b | From FBWA select QLOITER (gate running), then `rc 8 2000` (FBWA) before `DOOR_ENTRY_MS`. | Pending cancels; stays FBWA; no VTOL entry; doors close. |
| 6c | Select QHOVER (pending), then QLOITER before delay elapses. | Retargets; timer restarts; ends in QLOITER. |
| 6d | FW↔FW via selector (MANUAL/FBWA/CRUISE if mapped). | Immediate; doors stay closed. |

---

## 7. MANUAL override & lockout

| ID | Steps | Expect / Pass |
|----|-------|---------------|
| 7a | Hovering QLOITER, flip override to MANUAL. | Immediate MANUAL (bypasses script); lift motor stops, nacelles slew forward; doors stay open then close per 5a/5b. |
| 7b | While in MANUAL, move `rc 8` to a VTOL position. | `MANUAL override active - selector ignored`; **no** mode change, **no** door opening. |
| 7c | Override back to rest (→FBWA), then select QLOITER. | Selector works again → QLOITER via the normal gate. |
| 7d | FBWA → override MANUAL. | MANUAL; doors stay closed. |
| 7e | `param set DOOR_ENABLE 0` (or stop script), flip override to MANUAL. | MANUAL still engages (script-independent). Doors hold last PWM. Re-enable after. |
| 7f | From FBWA select QLOITER, then flip override to MANUAL before `DOOR_ENTRY_MS` elapses. | `mode changed, VTOL entry cancelled`; **stays MANUAL** — no VTOL entry when the delay would have ended. |

---

## 8. External / automatic mode changes

| ID | Steps | Expect / Pass |
|----|-------|---------------|
| 8a | In FBWA, `mode QHOVER` from GCS. | `VTOL entered externally - opening doors` (warning); doors open reactively; mode is QHOVER. |
| 8b | In QLOITER (selector on QLOITER), `mode FBWA` from GCS. | Mode goes FBWA and **stays** — script does not drag it back. Doors follow the transition. |
| 8c | AUTO mission with VTOL_TAKEOFF / VTOL_LAND. | Doors track phase; external-VTOL warning on the VTOL legs (reactive, documented). |
| 8d | RTL that auto-transitions to a VTOL land. | Doors open reactively during the back-transition + warning (known limitation). |
| 8e | From FBWA select QLOITER, then `mode RTL` from GCS before `DOOR_ENTRY_MS` elapses. | `mode changed, VTOL entry cancelled`; **stays RTL**. |

---

## 9. Ground & disarmed

| ID | Steps | Expect / Pass |
|----|-------|---------------|
| 9a | Disarmed in QHOVER (nacelles up). | Doors open (`DOOR_ST` 1/2). |
| 9b | Disarmed, `rc 8 2000` (FBWA); let nacelles settle forward. | Doors **close** (`DOOR_ST 0`) once `TILT_POS` reaches forward (`DOOR_TILT_FWD`) & `LIFT_PWM` low — the disarmed-close fix. |
| 9c | Disarmed in FBWA, select QHOVER. | Doors open + `DOOR_ENTRY_MS` gate, then QHOVER. Arm → VTOL spin-up only **after** doors open. |
| 9d | Reboot with `rc 8` on QHOVER position. | Gated open + delayed entry, not an instant blocked spin-up. |

---

## 10. Failure & robustness

| ID | Steps | Expect / Pass |
|----|-------|---------------|
| 10a | `param set DOOR_MODE_CH 0`. | Doors track phase reactively; no mode gating; no script errors. Restore after. |
| 10b | Trigger RC failsafe (e.g. `param set SIM_RC_FAIL 1`). | No selector edge acted on; failsafe mode engages; doors reactive; no misfire. Clear after. |
| 10c | Rapidly sweep the selector across all positions. | No stuck/oscillating state, no runaway mode commands. |
| 10d | `param set DOOR_TILT_FN 0` momentarily. | `TILT_POS`=0 → treated as "deployed" → doors stay open (fail-safe). Restore after. |
| 10e | `param set DOOR_DEBUG 0`. | Verbose `F35B[dbg]:` messages stop; production event messages remain. |
| 10f | Start SITL with RC failsafe active (`param set SIM_RC_FAIL 1`, reboot), selector on a VTOL position; then clear it (`SIM_RC_FAIL 0`). | No mode command while RC is invalid. Once RC is valid, the selector position is acted on through the normal gate (as in 9d). |

---

## Priority subset (if short on time)
**1a–1c, 2a, 3a, 3c, 4c, 5a, 5b, 6a, 7a, 7b, 7f, 9b** — covers the safety-critical gating (entry
delay, both close gates), the edge-trigger and latch fixes, and override independence.
