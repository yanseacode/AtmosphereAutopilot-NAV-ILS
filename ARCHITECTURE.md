# Release architecture

## Decision

AtmosphereAutopilot NAV/ILS is an independent AtmosphereAutopilot add-on licensed under GPLv3 or later. It registers a new AA high-level autopilot and uses AA's existing per-vessel flight model, director, adaptive axis controllers, standard fly-by-wire controller, and non-PID speed controller.

It does not modify AtmosphereAutopilot or link to the KramaxAutoPilot assembly. During development it can read an installed Kramax flight-plan file directly so both autopilots can be compared with identical data.

## License boundary

AtmosphereAutopilot is GPLv3-or-later. KramaxAutoPilot declares CC BY-NC-SA 4.0 and is derived from Pilot Assistant under that license. Creative Commons currently designates no non-CC license as compatible with BY-NC-SA 4.0. The former bridge directly linked both systems and is therefore retained only as a local research prototype.

The current maintainer has indicated that flight-plan reuse can be permitted. Before a public release bundles that data, retain a written grant covering redistribution, modification, attribution, and the intended release license. Until that is recorded, public packages should ship the compatible reader without Kramax's plan collection.

Public AtmosphereAutopilot NAV/ILS releases must follow these rules:

1. Do not copy Kramax or Pilot Assistant source. Bundle flight-plan data only under the maintainer's explicit written permission and required attribution.
2. Do not add a Kramax assembly reference, reflection adapter, or bundled compatibility bridge. Reading the documented ConfigNode plan format is an optional data compatibility feature.
3. Derive algorithms from public aviation geometry, KSP APIs, and independently written requirements.
4. Record the provenance and permission for every bundled airport and runway dataset.
5. Keep AtmosphereAutopilot as an external dependency and distribute AtmosphereAutopilot NAV/ILS source under GPLv3-or-later.

## Component boundaries

- `Core`: deterministic navigation geometry and guidance state. It has no Unity, KSP, AA, or Kramax dependency.
- `Ksp`: ConfigNode loading and conversion between KSP vessel state and core inputs.
- `Controller`: the sealed AA `StateController`, lifecycle, GUI, and calls to AA controllers.
- `Data`: native flight plans and independently generated runway records.
- `Tests`: executable offline checks for geometry, transitions, parsing, metadata, and forbidden dependencies.

## Runtime ownership

Only one AA high-level controller is active. When AtmosphereAutopilot NAV/ILS is selected, it owns the AA control callback. With no lateral or vertical guidance selected it delegates the complete attitude-control callback to Standard Fly-By-Wire and reports `FBW standby`.

Heading, NAV, vertical-speed, altitude, radar-altitude, and glide-slope guidance produce only a desired ground track and vertical flight path. The add-on converts those targets into a desired velocity vector and calls AA's `DirectorController` once per physics frame. The Director coordinates AA's pitch, roll, and yaw angular-velocity controllers; the add-on does not calculate a bank target, impose a bank limit, or write control surfaces on this path. The explicit Bank and Pitch modes remain separately identified `Direct AA` modes because their user-selected attitude targets cannot be represented by the Director's course-and-flight-path input.

NAV lateral guidance temporarily sets AA's actual `wing_leveler` setting off so the Roll Controller and Craft Settings windows show the same state. The prior value is restored when lateral guidance ends. Likewise, the add-on's speed mode temporarily turns AA's built-in speed-control toggle off to prevent two calls to the same thrust controller; the Prograde Thrust Controller GUI reflects that state and the prior value is restored afterward. No private shadow setting is used.

## Control-integration rule

AtmosphereAutopilot NAV/ILS computes navigation targets. AA decides how the aircraft reaches them. New guidance work must use Standard Fly-By-Wire, the Director, and AA's existing controller settings before introducing another control law.

Do not add a hard-coded bank angle, turn rate, command slew, control-authority cap, controller override, or silent protection toggle. If testing eventually proves that AA cannot provide a required behavior, document the failed AA options and consult the project owner before adding the smallest necessary new logic. Any setting changed by this add-on must change the real AA setting, remain visible in every AA GUI that exposes it, and be restored when the owning NAV/ILS mode disengages.

The first public version ends automatic guidance above the flare and leaves throttle, flare, touchdown, braking, steering, and thrust reversers to the player. Physics warp support is a release requirement: 2x minimum and 4x target, after controller behavior is measured at each rate.

## Planned delivery stages

1. Core and package skeleton, independent tests, and license guard. Completed in 0.1.
2. Native and Kramax-format plan loading, selection, visible legs, direct-to, and sequencing. Implemented in 0.3 development build.
3. Heading, altitude, per-axis FBW, and speed control through AA. Completed in 0.2 development build.
4. NAV leg interception. Implemented for flight validation; waypoint sequencing occurs on fix passage without a hidden fixed-rate turn model.
5. Active-leg NAV and GS guidance from Kramax vertical waypoint data, with a manual landing handoff. Implemented in 0.3 for flight validation.
6. Physics-warp validation and manual thrust-reverser compatibility.
7. Independently generated Kerbin airport/runway package, SpaceDock archive, and CKAN metadata.
