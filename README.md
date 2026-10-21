# AtmosphereAutopilot NAV/ILS

AtmosphereAutopilot NAV/ILS is a navigation and approach controller for AtmosphereAutopilot. Its release goal is to reproduce Kramax AutoPilot's familiar flight-plan, NAV, ILS, and mode workflow while using AtmosphereAutopilot's adaptive flight controls.

> **Development release:** AtmosphereAutopilot NAV/ILS is approaching public flight testing. Keep backups of craft and save files, and complete initial tests with stable aircraft at safe altitude.

## Requirements

- Kerbal Space Program 1.12.x
- [AtmosphereAutopilot](https://github.com/Boris-Barboris/AtmosphereAutopilot)
- [Kramax AutoPilot Continued](https://github.com/linuxgurugamer/KramaxAutoPilot)
  - Kramax AutoPilot flight-plan files are currently used as the airport database. Kramax does not need to control the aircraft and AtmosphereAutopilot NAV/ILS does not link to its plugin.

## Installation

1. Download the current development package from the repository's Releases page.
2. Copy its `GameData/AtmosphereAutopilotNav` directory into the KSP installation's `GameData` directory.
3. Keep AtmosphereAutopilot and Kramax Autopilot installed. Kramax plans are read from `GameData/KramaxAutoPilot/DefaultFlightPlans.cfg` and `FlightPlans.cfg` when present.
4. Start KSP and open **NAV / ILS** from AtmosphereAutopilot's module manager.

The current 0.7.0 development build connects the Kramax-style Bank, Heading, NAV, Pitch, Vertical Speed, Altitude, Radar Altitude, GS, Velocity, Acceleration, and Landing-speed modes to AtmosphereAutopilot's controllers. Heading, NAV, VSpd, Alt, RAlt, and GS supply desired course and flight-path targets to AA's Director, which coordinates AA's adaptive pitch, roll, and yaw controllers using the aircraft's configured settings. There is no add-on bank limit or fixed-rate turn model. Explicit Bank and Pitch selections are clearly reported as `Direct AA` modes. A shortcut row opens the AA pitch, roll, yaw, sideslip, thrust, director, and craft-settings windows directly from NAV / ILS. When Kramax is installed, AtmosphereAutopilot NAV/ILS reads `DefaultFlightPlans.cfg` followed by `FlightPlans.cfg`, applies user-plan name overrides in the same order, and exposes those plans in its own flight-plan manager. The manager can update or rename the active plan in Kramax's user-plan file, with a backup made before replacement. It does not link to or call the Kramax plugin.

NAV and GS now share one active plan leg, as they do in Kramax. Waypoint flags `Vertical`, `IAF`, `FAF`, `RW`, and `Stop` retain their Kramax meanings. Direct-to normally begins at the aircraft's present position, while direct-to FAF or runway preserves the published inbound leg. A leg must become valid before it can auto-sequence, and approach-critical legs are protected from early capture.

AA owns the control lifecycle. With lateral and vertical guidance off, the controller delegates the attitude callback to AA Standard Fly-By-Wire and reports `FBW standby`. NAV and vertical flight-path guidance use AA's Director; speed control uses AA's Prograde Thrust Controller. Landing-speed mode selects the configured IAF, FAF, or final speed from the active plan phase. The development build hands the aircraft back to FBW at 150 m radar altitude on the runway leg whether NAV, GS, or landing-speed guidance is active; flare, touchdown, braking, steering, and reversers remain manual.

## Building and testing

Run `build.ps1` from PowerShell to build and validate the plugin. Output is written to `artifacts`. The project expects a KSP development checkout at the parent directory, including the KSP managed assemblies and AtmosphereAutopilot DLL used as compile-time references.

See `TESTING.md` for the current flight test and `KRAMAX_PARITY.md` for the release checklist. Please include `KSP.log`, the aircraft used, selected plan, active modes, and reproduction steps with a flight-control report.

## License and data

AtmosphereAutopilot NAV/ILS is licensed under GPL-3.0-or-later. AtmosphereAutopilot and Kramax AutoPilot remain separate projects. See `PROVENANCE.md` for the code and flight-plan data boundary.
