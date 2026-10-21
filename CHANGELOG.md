# Changelog

This project is still in development. Development packages are published so that navigation behavior can be flight-tested before the first stable release.

## 0.7.0-dev - 2026-10-21

### Corrected AA control integration

This release corrects a control-path mistake in the 0.6.x development builds. While tuning course capture, those builds gradually added local turn anticipation, command slew, a fixed 25-degree bank limit, and direct roll, pitch, and yaw commands. That improved one test aircraft on one route, but it bypassed part of AtmosphereAutopilot's adaptive Fly-By-Wire design and made the displayed `FBW standby` state misleading.

The normal guidance modes now follow the intended architecture:

1. NAV/ILS computes the requested heading, course, altitude, vertical speed, speed, and flight path.
2. Heading, NAV, VSpd, Alt, RAlt, and GS turn those requests into a desired velocity vector.
3. AtmosphereAutopilot's Director receives that vector once per physics frame.
4. The Director coordinates AA's pitch, roll, and yaw angular-velocity controllers, which command the aircraft.

The hidden 25-degree bank limit, fixed-rate waypoint turn calculation, lateral and vertical command slew, and silent changes to AA's thrust PID, brake use, and pitch/yaw moderation have been removed. Aircraft response once again comes from the visible AA Director, controller, and craft settings. The explicit Bank and Pitch modes remain labelled `Direct AA` because they command user-selected attitude targets that the Director's course-and-flight-path interface cannot represent.

NAV/ILS still disables AA's real wing-leveler setting while lateral guidance owns the course, and disables AA's real Standard Fly-By-Wire speed toggle while NAV/ILS speed control is active. These are visible AA settings; their previous values are saved and restored when the corresponding mode disengages. Guidance telemetry now records control ownership and the relevant AA state for flight-test diagnosis.

Other changes in this development series include the Kramax-style NAV/ILS interface, Kramax flight-plan compatibility, shared NAV/GS leg handling, protected approach-leg sequencing, Director and controller window shortcuts, deviation indicators, mode-specific target increments, and manual FBW handoff before landing.

Anyone testing an earlier 0.6.x package should replace it with 0.7.0-dev before reporting handling or guidance behavior.

## 0.6.5-dev and earlier

Early interface, flight-plan, mode-control, and guidance prototypes. These packages established the Kramax-compatible workflow but do not represent the intended final AA control architecture. They are retained in source history for development traceability and should not be used for current flight testing.
