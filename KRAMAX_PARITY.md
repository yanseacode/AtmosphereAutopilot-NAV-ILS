# Kramax compatibility target for Navigation

The release target is a familiar replacement for Kramax AutoPilot's airplane workflow, backed by AtmosphereAutopilot control laws.

| Area | Kramax behavior to preserve | Status |
|---|---|---|
| Plan files | Read default and user Kramax ConfigNode plans; user plans override defaults by planet and name | Implemented in 0.3 |
| Plan manager | Planet, current name/info, refresh, available-plan list, load plan | Implemented for testing |
| Waypoint table | Active/direct-to, location, altitude, heading, distance, IAF/FAF/Final/RW roles, compact rows without scrollbars | Implemented for testing |
| Direct-to | Present-position intercept for ordinary fixes; preserve inbound leg for FAF and runway | Implemented in 0.3 |
| Sequencing | Require a valid leg before sequencing; protect FAF, runway, and stop from early turns | Implemented in 0.3; turn anticipation needs matching |
| Lateral modes | Bank, heading, NAV, independent roll enable | Connected in 0.5; Bank requires flight validation |
| Vertical modes | Pitch, vertical speed, altitude, radar altitude, GS, independent vertical enable | Connected in 0.5; new modes require flight validation |
| Throttle modes | Velocity, acceleration, landing speed schedule | Connected in 0.5; Acc and Land require flight validation |
| Approach speeds | IAF, FAF, and final values driving landing throttle mode | Connected in 0.5; plan-phase changes require flight validation |
| Landing automation | Gear, throttle cut, flare/touchdown, brakes | Tabled; shared NAV/GS/Land 150 m FBW handoff implemented |
| Plan editing | Rename/update and write user plan file | Implemented in 0.6; runtime validation pending |
| NAV/GS indicators | Navball-adjacent deviation bars and signed metre labels | Implemented in 0.5.2; visual/runtime comparison pending |
| External CDI | Compatibility with the separate CDI mod | Deferred final compatibility phase |
| Warp | Stable control at 2x, 4x target | Pending runtime validation |
| Reversers | Manual operation must remain available after handoff | Pending runtime validation |

No item is considered complete for release until its behavior has been compared in flight against Kramax using the same plan and aircraft.
