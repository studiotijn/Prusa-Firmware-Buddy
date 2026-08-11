# Bookmark3D — Design

## 1. Problem

Print logos, project names, and decorative reliefs directly onto hardcover
book covers, notebooks, and similar rigid-but-lightly-compressible objects,
using an otherwise-stock Prusa MK4. The printer's existing loadcell-based
probing assumes a rigid bed and hard contact; a book cover is compressible
enough that the standard threshold/hysteresis contact model risks either
false triggers (too shallow) or crushing the cover (too much force before
trigger).

All changes must be:

- **Modular** — isolated behind a single build-time flag, off by default.
- **Additive** — zero behavior change for standard MK4 builds/users.
- **Reversible** — no destructive edits to shared Marlin/Prusa logic; new
  code paths, not rewrites, wherever possible.

## 2. Scope of firmware changes

Based on a codebase survey (2026-07-31), the relevant subsystems and their
current implementation are:

| Area | Key files | Current behavior |
|---|---|---|
| Loadcell driver | `src/common/loadcell.hpp/.cpp` (`class Loadcell`) | Raw sample ingestion (`ProcessSample`), IIR bandpass filtering (`BandPassFilter<ZFilterParams>`/`<XYFilterParam>`), taring (`Tare`, `TareMode::Static/Continuous`), fixed per-printer thresholds (`thresholdStatic`, `thresholdContinuous`, `hysteresis`), endstop state (`GetMinZEndstop`), and probe-abort safety (`arm_probe_safety`, `probe_safety_stop`, `HomingSafetyCheck`). |
| Contact-point analysis | `src/common/probe_analysis.hpp/.cpp` (`class ProbeAnalysisBase`/`ProbeAnalysis<WindowSize>`) | Post-processes the windowed force curve around a touchdown via `Analyse(bool is_nozzle_clean)` to refine the exact trigger point. **Primary hook for a compressible-surface contact model.** |
| Probing | `lib/Marlin/Marlin/src/module/probe.cpp/.h` (`class Probe::run_z_probe`) | Marlin probe module, patched to read `loadcell.GetMinZEndstop()` instead of a physical microswitch. |
| Homing | `lib/Marlin/Marlin/src/module/prusa/homing_cart.cpp`, `homing_corexy.cpp`, `homing_modus.cpp`, `homing_utils.cpp` | Precise-homing kinematics consuming loadcell endstop state for Z. |
| Mesh bed leveling | `lib/Marlin/Marlin/src/feature/bedlevel/bedlevel.cpp/.h`, `.../ubl/ubl.cpp`, `ubl_G29.cpp` | Unified Bed Leveling; each mesh probe point calls into `probe.cpp`/loadcell. |
| First-layer calibration | `src/common/selftest/selftest_firstlayer.cpp/.hpp` (+ `_interface`, `_config`), GUI: `src/gui/wizard/selftest_frame_firstlayer*.cpp/.hpp` | State machine + wizard driving the first-layer test print and user squish feedback, feeding back into Z-offset. |
| Bed heating | `lib/Marlin/Marlin/src/gcode/temperature/M140_M190.cpp` | Standard Marlin `M140`/`M190` handling; heater state in `lib/Marlin/Marlin/src/module/temperature.*`. |
| Printer/bed variant config | `include/common/printer_model.hpp/_data.hpp`, `src/common/printer_model.cpp` | `PrinterModel` enum + metadata table. No existing "bed type" enum — closest analog is sheet-profile support (`HAS_SHEET_PROFILES`). |
| Build-time feature flags | `ProjectOptions.cmake`, `cmake/Options.cmake` | `define_boolean_option(NAME value)`, `set_feature_for_printers(...)`; each option generates `include/option/<name>.h` with a `HAS_X()`/`IS_X()` macro. This is how `HAS_LOADCELL`, `HAS_SHEET_PROFILES`, etc. are gated per printer. |
| Runtime user toggles | `src/gui/screen_menu_experimental_settings.hpp` (+ `_release.cpp`, `_debug.cpp`), `src/gui/MItem_experimental_tools.*` | Existing GUI pattern for exposing experimental settings to users at runtime. |
| Persisted settings | `src/persistent_stores/store_instances/config_store/store_definition.hpp`, `defaults.hpp`, `migrations.cpp/.hpp`, `store_instance.hpp/.cpp` | Versioned EEPROM-backed config store, accessed as `config_store().<field>.get()/set()`. Idiomatic place for a new persisted "soft surface mode" flag + tunables. |
| Self-test/calibration | `src/common/selftest/selftest_loadcell*.cpp/.hpp` | Existing loadcell self-test/calibration routines run during setup. |

## 3. Feature-flag strategy

Add one new build option in `ProjectOptions.cmake`:

```cmake
define_boolean_option(HAS_SOFT_SURFACE_MODE OFF)
```

Not attached to any printer via `set_feature_for_printers(...)`, so it stays
`0` (disabled) in every stock build. Generates `include/option/has_soft_surface_mode.h`.
All new code is wrapped:

```cpp
#include <option/has_soft_surface_mode.h>
#if HAS_SOFT_SURFACE_MODE()
// ...
#endif
```

This guarantees standard MK4 firmware behavior is bit-for-bit unchanged
unless a developer build explicitly opts in — satisfying the "standard
functionality fully preserved" goal without runtime branching cost in
default builds.

## 4. Soft Surface Mode — architecture

### 4.1 Persisted config (config_store)

New fields in `store_definition.hpp`, guarded by `HAS_SOFT_SURFACE_MODE()`:

- `bed_type: BedType` — `standard | soft_surface` (added in branch 4, §4.5)
- `soft_surface_mode_enabled: bool`
- `soft_surface_probe_force: float` — gentle contact-detection threshold (lower magnitude than the stock compiled-in threshold)
- `soft_surface_probe_speed: float` — Z feedrate during the slow probe move (slower than stock; wired up in branch 6 after being unused since branch 1)
- `soft_surface_probe_samples: uint8_t` — number of samples to average per point
- `soft_surface_filter_strength: float` — extra low-pass coefficient applied on top of the existing bandpass filter (not yet wired up — see open questions)
- `soft_surface_compression_compensation: float` — fixed offset (mm) subtracted from the averaged trigger height to correct for known cover indentation
- `soft_surface_max_probe_force: float` — hard abort ceiling (grams), distinct from and above `soft_surface_probe_force`; added in branch 6, §4.7
- `soft_surface_max_indentation: float` — max acceptable sample spread (mm) at one point before the batch is rejected as unreliable; added in branch 6, §4.7

Defaults added in `defaults.hpp`. No `migrations.cpp` entry needed: the
config_store is a hash-keyed journal (each `StoreItem`'s `journal::hash("...")`
name is its key), so a brand-new field simply isn't found on old EEPROM data
and falls back to its default — `migrations.cpp` is only for renaming/retyping
an *existing* field. (Corrected from the original draft of this doc, which
assumed a migration entry would be needed.)

### 4.2 Loadcell / contact detection

`Loadcell` gains a runtime-checked branch (not a new constant threshold)
that, when `soft_surface_mode_enabled` is true, substitutes
`soft_surface_probe_force`/`filter_strength` for the compiled-in
`thresholdStatic`/`hysteresis`. Detection changes from single hard-threshold
crossing to **force-buildup-based detection**: trigger fires once the force
signal's slope sustains above a lower bound for N consecutive samples (a
new mode in `probe_analysis.cpp`'s `Analyse()`), rather than one instantaneous
threshold crossing — reduces false triggers from noisy, soft materials
while still catching genuine contact early enough to avoid crushing the cover.

### 4.3 Probing loop (multi-sample averaging)

`Probe::run_z_probe` gains a soft-surface path that repeats the probe at a
given XY point `soft_surface_probe_samples` times at `soft_surface_probe_speed`,
and averages the resulting trigger heights (with outlier rejection —
reuse the existing `ProbeAnalysisBase` statistics helpers) before returning
a single Z value to the MBL/UBL grid-fill caller. This directly answers the
"reproducibility" requirement: a single noisy sample never determines mesh
geometry.

### 4.4 Compression compensation

After averaging, `soft_surface_compression_compensation` is subtracted from
the final trigger height, to correct for the book cover's known indentation
under probe force. Starting as a single fixed, user-tunable value (measured
empirically per material during calibration — see `docs/calibration.md`);
an adaptive/force-proportional model is a possible future iteration, not
required for v1.

### 4.5 Bed type: "Soft Surface (Bookmark3D)"

Modeled as a config_store-persisted enum (`bed_type: Standard | SoftSurface`),
independent of `PrinterModel` (this is a per-print-session choice, not a
hardware variant). When `bed_type == SoftSurface`:

- `M140`/`M190` (`lib/Marlin/Marlin/src/gcode/temperature/M140_M190.cpp`)
  short-circuit to a no-op success when guarded by
  `HAS_SOFT_SURFACE_MODE()` and the active bed type — no heater PID engaged,
  no wait, no timeout error.
- The generated `Bookmark3D.ini` PrusaSlicer profile sets bed temperature
  to 0°C, so in practice the firmware path is rarely exercised, but the
  no-op guard makes the firmware itself robust to a stray `M140 S60` in
  hand-written or third-party G-code.

### 4.6 First-layer calibration

Revised after reading the actual wizard implementation (`selftest_firstlayer.cpp`,
`selftest_frame_firstlayer_questions.cpp`): there is no algorithmic squish-tolerance
check to relax. The wizard's live-Z step is entirely manual — the user watches the
extruded test line and turns the knob — so it needs no surface-specific logic; that
part is already surface-agnostic.

What the wizard *does* do that's specific to Soft Surface Mode: it calls
`preheat()` (in `selftest_firstlayer.cpp`), which — like the general preheat/filament
paths in `M70X_preheat.cpp` (`filament_gcodes::preheat_to`, `M1700_preheat`) — calls
`thermalManager.setTargetBed(...)` **directly**, bypassing the `M140`/`M190` no-op
guard added in §4.5/branch 4. Left unpatched, `stateWaitBed()` would wait on a bed
target that will never be reached even with `bed_type == soft_surface`. Fixed by
guarding all three direct call sites the same way as `M140`/`M190`. No change to
`soft_surface_compression_compensation` handling was needed here — it's applied in
`run_z_probe()` (§4.3/§4.4), which this wizard already calls into via mesh bed
leveling (`G29`, `stateMbl()`), so it's covered automatically.

**Correction found by actually compiling with `HAS_SOFT_SURFACE_MODE=ON`
(2026-08-07):** `selftest_firstlayer.cpp` is only built for
`PRINTER STREQUAL "MINI"` or `"MK3.5"` (`src/common/selftest/CMakeLists.txt:43-45`)
— it's not part of the MK4 build at all. MK4 has no dedicated first-layer
selftest state machine; `SelftestFrameFirstLayer` (`src/gui/wizard/selftest_frame_firstlayer.cpp`)
is GUI-only display logic with no preheat call of its own. So on the printer
this project actually targets, the `preheat()` guard above is dead code — it's
correct and harmless, just unreachable for the MK4+`HAS_SOFT_SURFACE_MODE`
combination that matters. This doesn't need fixing: MINI/MK3.5 have no
loadcell either (§3/`HAS_LOADCELL` excludes them), so Soft Surface Mode
wouldn't make sense there regardless — it reinforces the §8 open question
that `HAS_SOFT_SURFACE_MODE` should probably be restricted to MK4 builds
outright.

### 4.7 Safety

As implemented (revised from the original plan after reading the real abort/retry
plumbing already in `loadcell.cpp`/`probe.cpp`). All new checks guarded by
`HAS_SOFT_SURFACE_MODE()`:

- **Max probe force** — new `soft_surface_max_probe_force` config field (hard
  ceiling, grams). In `Loadcell::ProcessSample()`, if force exceeds it before
  contact is confirmed, calls the existing `probe_safety_stop()` immediately
  (quick_stop + `probe_safety_tripped`), bypassing `confirm_samples` entirely —
  continuing the descent for several more samples once already over this
  ceiling risks crushing the cover. Reuses `run_z_probe()`'s existing
  `probe_safety_did_trip()` / `handle_probe_safety_trip()` retry path; no new
  abort plumbing was needed.
- **Max indentation** — new `soft_surface_max_indentation` config field (mm).
  `run_z_probe()` tracks the min/max Z of accepted samples per point and
  rejects the averaged result if the spread exceeds it — a rigid surface
  repeats consistently, so a wide spread means the surface is compressing
  unpredictably. This doubles as the **"surface too soft" warning** (logged via
  `SERIAL_ECHOLNPAIR_F`); a full GUI-level warning is left to the
  experimental-settings branch (§5), consistent with that branch owning
  user-facing messaging.
- **Probing timeout** — already covered by the existing `HomingSafetyCheck()`
  stale-loadcell-data check and `do_probe_move()`'s distance limit
  (`z_probe_low_point`); both apply unconditionally, no new code needed.
- ~~Force-curve anomaly detection in `ProbeAnalysisBase::Analyse()`~~ — not
  implemented as originally planned. The min/max spread check above catches
  the same "inconsistent/too-soft surface" failure mode without touching the
  complex windowed statistical engine, for the same reason branch 2 left it
  alone (§4.2).
- **Experimental warning** — the experimental-settings menu entry
  (`screen_menu_experimental_settings`) and the GUI wizard both surface an
  explicit "Soft Surface Mode is experimental" notice before first use.

### 4.8 Z-axis safe homing

Not in the original plan — found while reviewing the homing path, not
probing. `home_z_safely()` (`lib/Marlin/Marlin/src/gcode/calibrate/G28.cpp`,
guarded by `Z_SAFE_HOMING`) moves to a fixed, bed-relative
`Z_SAFE_HOMING_X_POINT`/`_Y_POINT` before homing Z — that point sits over the
bare-metal calibration dot, off-bed, which is correct when homing against a
bare heated bed but wrong here, since it's very unlikely to land on the book
cover itself. When `soft_surface_mode_enabled` is set, `home_z_safely()` now
homes to the center of the slicer-provided print area
(`PrintArea::get_bounding_rect()`, populated by `M555` in `start_gcode` ahead
of `G28`) instead. The Z-axis homing bump move also triggers off the same
loadcell contact detection as probing (`homeaxis(Z_AXIS)` → the same ISR path
`probe_at_point()` uses), so it's wrapped in the same
`Loadcell::SoftSurfaceModeEnabler` at the same call site pattern as
`probe.cpp`.

### 4.9 Live probing-force gauge (GUI)

Added 2026-08-11, after the original 8-branch plan. Shows a live loadcell-force
bar + adjustable-threshold arrow on the LCD for the duration of one
soft-surface probing session (G28's homing bump and G29's mesh-probing pass
each open their own session).

- **Live force display**: reuses the existing `sensor_data().loadCell`
  channel (`src/common/sensor_data.hpp`), already written every sample inside
  `Loadcell::ProcessSample` (§4.2) — no new core→GUI plumbing needed for the
  read side, this is the same channel `MI_INFO_LOADCELL` already polls.
- **Live threshold override**: new `Loadcell::SetLiveProbeForceOverride()` /
  `GetEffectiveProbeForce()` (`loadcell.hpp`/`.cpp`), backed by two lock-free
  atomics — the reverse-direction counterpart to `sensor_data().loadCell`
  (GUI thread writes, Marlin thread reads). `probe_at_point()`
  (`probe.cpp`) and `home_z_safely()` (`G28.cpp`) already re-read
  `config_store().soft_surface_probe_force` fresh on *every* probe/homing
  attempt (not just once per G29 call), so intercepting that read is all
  that's needed for knob adjustments to take effect on the very next
  attempt within the same session — no per-tick config_store writes (bad for
  flash wear).
- **Session lifetime**: a new `ClientFSM::SoftSurfaceProbing` /
  `PhaseSoftSurfaceProbing` (single phase, no buttons — mirrors `ClientFSM::Wait`
  exactly), opened via `marlin_server::FSM_Holder` at the top of
  `unified_bed_leveling::probe_major_points()` (spans the *entire* G29 grid
  loop, not re-opened per point) and around the homing-bump block in
  `home_z_safely()`. `DialogSoftSurfaceProbing`
  (`src/gui/dialogs/dialog_soft_surface_probing.hpp/.cpp`) is bound to it via
  `DialogHandler.cpp`'s `FSMDisplayConfig` table, same mechanism as
  `window_dlg_wait_t`/`ClientFSM::Wait`.
- **Save on leaving the session**: `DialogSoftSurfaceProbing`'s destructor
  writes the live-adjusted value to `config_store().soft_surface_probe_force`
  and clears the override — mirrors `WindowLiveAdjustZ::~WindowLiveAdjustZ()`'s
  `Save()`-on-destroy pattern (`src/gui/dialogs/liveadjust_z.cpp`), which this
  whole feature is modeled on (vertical scale line, live marker position,
  knob-driven `GUI_event_t::KNOB` adjustment).
- **Widget**: `WindowSoftSurfaceGauge` (short vertical baseline, a filled
  vertical bar for the live force, a hand-drawn triangle arrow for the
  threshold — no vertical-arrow icon resource exists in this codebase).
  Visual scale tops out at `soft_surface_max_probe_force` (§4.7's existing
  hard-abort ceiling), so the bar can never exceed the height the printer
  would itself abort probing at.
- **Shared bounds**: `soft_surface_probe_force_spin_config`
  (min/max/step/decimals) moved out of `MItem_experimental_tools.cpp` into
  `src/gui/soft_surface_probe_force_config.hpp` so the Experimental Settings
  menu item and this live overlay can't drift apart on bounds.
- Compile-verified only (`HAS_SOFT_SURFACE_MODE=OFF` confirmed byte-identical
  to the pre-existing baseline `.bbf`; `=ON` debug builds clean) — like every
  other Soft Surface Mode branch to date, not yet tested on real hardware.
  Widget pixel geometry (`gauge_rect()` in `dialog_soft_surface_probing.cpp`)
  is a first pass; needs visual tuning against a real display or the
  simulator, not verifiable in this headless build environment.
- Adding the new `ClientFSM` value required touching every place that
  exhaustively switches/tables over `ClientFSM` (compile-time-checked, not
  just convention): `src/common/marlin_server_types/client_response.cpp`'s
  `fsm_phase_responses` table, `src/common/fsm_states.cpp`'s FSM-priority
  `score()` switch, and both `ClientFSM` switches in
  `src/state/printer_state.cpp`. Worth remembering if another new `ClientFSM`
  is ever added to this fork.

## 5. UI / user toggle

As implemented: new items added to both `screen_menu_experimental_settings_debug`
and `_release` (identical set for this feature — unlike other items in that
menu, Soft Surface Mode isn't printer/build-variant-specific), all wrapped in
`#if HAS_SOFT_SURFACE_MODE()`, in `MItem_experimental_tools.hpp`/`.cpp`. No new
top-level menu — reuses the existing `ScreenMenuExperimentalSettings` entry
point, consistent with how other experimental features (fast draw, alt fan
correction) are surfaced today.

- `MI_SOFT_SURFACE_MODE_ENABLE` — `soft_surface_mode_enabled` toggle
  (`WI_ICON_SWITCH_OFF_ON_t`), writes to config_store **immediately** on
  change (mirrors `MI_FAST_DRAW_ENABLE`/`MI_AUTO_RETRACT_ENABLE`, not the
  deferred-`Store()` pattern below). Turning it *on* shows a `MsgBoxWarning`
  ("Soft Surface Mode is experimental... Continue?", default-focused on "No")
  and reverts the toggle if declined — this is the "experimental warning"
  called for in §4.7, modeled on `MI_AUTO_RETRACT_ENABLE`'s revert-on-decline
  structure.
- The seven numeric tunables (`soft_surface_probe_force`,
  `soft_surface_max_probe_force`, `soft_surface_probe_speed`,
  `soft_surface_probe_samples`, `soft_surface_filter_strength`,
  `soft_surface_compression_compensation`, `soft_surface_max_indentation`) —
  each a `WiSpin`-based `MI_SOFT_SURFACE_*` item with a deferred `Store()`,
  following the existing `MI_Z_AXIS_LEN`/`MI_CURRENT_X`-style pattern: the
  value only reaches config_store when the user confirms
  "Save and return" → reboot in `clicked_return()`. Not strictly required for
  correctness (all seven are already read live from config_store at
  probe/preheat time per §4.2–§4.4/§4.7), but `WiSpin` has no
  after-edit-commit hook to write immediately, and gating tunable changes
  behind an explicit confirm-and-reboot is arguably desirable anyway for
  safety-relevant probing parameters. Added to `ExperimentalSettingsValues`'s
  change-detection struct so editing only these fields still triggers the
  save prompt (note: `MI_LOADCELL_SCALE` is *not* tracked there today, which
  looks like a pre-existing oversight in that struct — left alone, out of
  scope for this branch).
- `soft_surface_filter_strength` is exposed here even though nothing reads it
  yet (§4.2 already flagged this as unwired) — kept for forward
  compatibility once loadcell signal filtering is implemented; not a
  regression introduced by this branch.

## 6. Out of scope for v1

- Adaptive/force-proportional compression compensation (fixed value only).
- Automatic surface-type detection (cardboard vs. linen vs. faux leather) —
  user selects tunables per material manually via calibration.
- Any structural hardware change (new sensors, modified probe tooling).
- Slicer-side (PrusaSlicer C++ source) changes — the print profile is a
  plain `.ini` (see `profiles/Bookmark3D.ini`, not yet created), not a
  slicer code fork.

## 7. Branching plan

All work happens on feature branches off `upstream/master`, never directly
on `master`, per `../CLAUDE.md`. Proposed sequence:

1. `feature/soft-surface-config-store` — §4.1, build flag (§3).
2. `feature/soft-surface-loadcell-detection` — §4.2.
3. `feature/soft-surface-probe-averaging` — §4.3, §4.4.
4. `feature/soft-surface-bed-type` — §4.5.
5. `feature/soft-surface-firstlayer-wizard` — §4.6.
6. `feature/soft-surface-safety-checks` — §4.7.
7. `feature/soft-surface-experimental-menu` — §5.
8. `feature/soft-surface-safe-homing` — §4.8. Not part of the original plan;
   added after discovering the homing path had the same off-bed-fixed-point
   problem the probing branches already solved.

Each branch should be small enough to review independently and must build
cleanly with `HAS_SOFT_SURFACE_MODE=OFF` (default, no change) and `=ON` (dev
build) before merging into a project integration branch.

## 8. Open questions

- Which physical printer variant(s) should even be allowed to enable
  `HAS_SOFT_SURFACE_MODE` in a dev build — MK4 only, or also MK4S/XL? (Affects
  `set_feature_for_printers(...)` scoping once the flag leaves "no printers".)
  Recommendation: MK4 only for now.
- Should compression compensation eventually read from a per-material
  preset table instead of a single manual value? Deferred to v2 per §6.
- Exact numeric defaults for `soft_surface_probe_force` /
  `..._probe_speed` / max indentation need empirical calibration against
  real hardcover samples — tracked in `docs/calibration.md` (not yet written).
- `soft_surface_filter_strength` is still declared but unused (same gap
  `soft_surface_probe_speed` had until branch 6): no code yet applies extra
  low-pass filtering on top of `Loadcell`'s existing bandpass filter. Needs
  a decision on *how* — a second filter stage in `BandPassFilter`, or a
  simple exponential smoothing pass on `filtered_z_load` — before wiring it
  up; deferred rather than guessed at.
- §4.9's live probing-force gauge (`ClientFSM::SoftSurfaceProbing`) was given
  priority score 2 in `fsm_states.cpp`'s `score()` (same tier as
  `Load_unload`/`Preheat`/`PrintPreview` — shows over the base
  `Printing`/`Selftest` screen, e.g. during the first-layer wizard's G28/G29
  sub-steps, but still yields to `SafetyTimer`(4)/`Warning`(5)). This is a
  judgment call, not derived from an existing precedent for an
  interactive-but-passive overlay; may need revisiting once real nested-FSM
  scenarios (e.g. a warning firing mid-probing-session) are actually
  exercised on hardware.
- §4.9's gauge widget geometry (`gauge_rect()`) is placed in the screen's
  upper-right corner by rough estimate only — never visually confirmed
  against a real display or the simulator (this environment is headless).
