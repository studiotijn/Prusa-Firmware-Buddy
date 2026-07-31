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

- `soft_surface_mode_enabled: bool`
- `soft_surface_probe_force: float` — max force before abort (lower than stock threshold)
- `soft_surface_probe_speed: float` — Z feedrate during probing (slower than stock)
- `soft_surface_probe_samples: uint8_t` — number of samples to average per point
- `soft_surface_filter_strength: float` — extra low-pass coefficient applied on top of the existing bandpass filter
- `soft_surface_compression_compensation: float` — fixed offset (mm) subtracted from the detected trigger point to correct for known cover indentation

Defaults added in `defaults.hpp`; a migration entry added in `migrations.cpp`
per the store's existing versioning convention — new fields never reuse or
reinterpret an existing EEPROM slot.

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

### 4.7 Safety

New checks, all guarded by `HAS_SOFT_SURFACE_MODE()`:

- **Max probe force** — hard abort if `soft_surface_probe_force` is exceeded
  before a trigger is detected (protects the cover from crushing).
- **Max indentation** — abort if the averaged trigger height implies more
  indentation than a configured ceiling (signals a surface too soft to
  probe reliably at all).
- **Probing timeout** — reuses/extends the existing `HomingSafetyCheck`
  pattern in `loadcell.cpp` so a stalled or noise-only signal aborts rather
  than hanging.
- **Force-curve anomaly detection** — flags multi-modal or non-monotonic
  force curves from `ProbeAnalysisBase::Analyse()` as unreliable and aborts
  instead of guessing.
- **Experimental warning** — the experimental-settings menu entry
  (`screen_menu_experimental_settings`) and the GUI wizard both surface an
  explicit "Soft Surface Mode is experimental" notice before first use.

## 5. UI / user toggle

A new menu item under `ScreenMenuExperimentalSettings` (`_release.cpp` for
release builds guarded by `HAS_SOFT_SURFACE_MODE()`, always present in
`_debug.cpp` builds) exposes: mode enable/disable, and the five tunables in
§4.1, via `MItem_experimental_tools`-style widgets. No new top-level menu —
consistent with how other experimental features are surfaced today.

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
