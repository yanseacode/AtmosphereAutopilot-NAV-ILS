# AtmosphereAutopilot NAV/ILS 0.6.3 guidance-response test

Deploy the test build to a separate KSP 1.12.x test installation. Leave the older Kramax/AA bridge disabled and select **NAV / ILS** in AtmosphereAutopilot.

## Main interface

1. Confirm the main window is approximately as wide as Kramax and contains the three long section buttons: `Roll`, `Vertical`, and `Auto-Throttle`.
2. Confirm the mode rows show `Bank / Hdg / NAV`, `Pitch / VSpd / Alt / RAlt / GS`, and `Vel / Acc / Land`.
3. Confirm stacked `+` / `-` buttons change only the pending field: Bank/Pitch 1, Hdg 5, VSpd/Vel/Acc 10, Alt 500, RAlt 100, approach speeds 5. Repeat a click to verify increments accumulate.
4. With a mode active, type or increment a target and verify the aircraft continues using the previous target until its left-hand target button is clicked. Check approach speeds the same way. Switching modes must not apply pending edits.
5. Confirm `Status:` precedes the status, section labels are white, and enabled sections include `(Active)`.
6. Drag each control section's bottom bar upward to hide its mode buttons; drag downward to reveal them. Targets remain accessible and guidance stays engaged. Confirm the window still drags and closes after resizing.
7. Every displayed mode is connected. Selecting a mode while its section is active should change mode without disengaging the section.
8. Confirm `Flight Plan`, `Load/Store`, the plan information and waypoint table, and the collapsible `Approach Speeds` panel fit without clipping.

## Kramax plan compatibility

1. Open **Load/Store**. The manager should list the same Kerbin plans as Kramax, including `KSC ILS 09`, `KSC DEP 09`, and both KSC circuit plans. The separate `KSC Development Circuit` entry is temporary native test data.
2. Load the same plan in each controller, one controller at a time. Confirm the plan name, description, waypoint order, altitude, heading, distance, and IAF/FAF/Final/RW labels agree.
3. In AtmosphereAutopilot NAV/ILS, click `D->` on an ordinary waypoint. It should become `Active` and NAV should fly from the aircraft's present position to that waypoint.
4. Click `D->` on a FAF or runway waypoint. The active course must remain the published preceding-waypoint-to-selected-waypoint leg. This is required for ILS alignment.

## Plan update and rename

1. Make a backup copy of `GameData/KramaxAutoPilot/FlightPlans.cfg`, then load a user plan.
2. In **Load/Store**, edit `Current plan info` and click `Update/Rename`. Confirm the yellow `Flight plan updated` message appears and the edited description remains after **Refresh**.
3. Change `Current plan name`, click `Update/Rename`, and confirm the renamed plan appears in `Available Plans`, loads normally, and remains after restarting KSP.
4. Confirm `FlightPlans.cfg.bak` contains the file as it existed immediately before the most recent save. Default plans must remain available and plans for other planets must remain in the user file.
5. Leave the name empty and click `Update/Rename`; confirm no file is written and `Flight plan name is required` appears.

## NAV and sequencing

1. Load `KSC ILS 09 Circuit (Small)` and select `NAV` with `ALT` at a safe circuit altitude.
2. Confirm the active row changes as each fix is passed. A plan loaded while the aircraft is already beyond several fixes must not cascade through them.
3. Use direct-to on a later ordinary waypoint and confirm it captures from the current position without a hard roll.
4. Verify all waypoint rows are visible without scrollbars, aligned in compact green fields, and every direct-to button works. Test both KSC circuits and a longer plan. Very long plans can exceed screen height; report any such plan for the next layout pass.
5. At an ordinary circuit corner, confirm the active leg advances early enough to begin a standard-rate turn rather than waiting until the aircraft is within 750 m of the fix. FAF, runway, and stop legs must still remain protected from early sequencing.
6. While coupled NAV and GS are active, confirm the aircraft turns with the normal AA director authority. The telemetry should show commanded track converging promptly instead of remaining 15-30 degrees behind through the entire turn.

## Individual control modes

1. At safe altitude, test `Bank` at small targets such as 10 and -10 degrees. Confirm the sign is correct and the aircraft settles without oscillation.
2. Test `Pitch` at small targets such as 5 and 0 degrees. Confirm this holds aircraft pitch attitude while Roll remains independently selectable.
3. Test `VSpd` with modest climb and descent values. Confirm the displayed actual vertical speed follows the target.
4. Test `RAlt` over gently varying terrain at a safe target. Do not use it near steep terrain in this development build.
5. Test `Acc` first at 0, then at small positive and negative values entered manually. Kramax uses increments of 10 for Acc; type smaller values for this flight test and click Target Acc to apply.
6. Load an approach plan and select `Land`. Confirm the displayed auto-land speed changes from the configured IAF speed to FAF speed and then final speed as the active waypoint role changes.

## ILS and handoff

1. Load `KSC ILS 09`. Use `D->` to select the IAF or FAF as appropriate for the aircraft's position, then select `NAV`.
2. Select `GS` only when the active leg has vertical guidance. GS should follow the altitude profile stored in the same Kramax leg; there is no separate LOC mode. NAV/GS deviation bars appear around the navball when those modes are selected.
3. On the FAF-to-runway leg, NAV supplies lateral guidance and GS supplies vertical guidance. The active row should be the runway/Final waypoint.
4. At 150 m radar altitude, NAV, GS, and Land speed control should disengage together and ordinary AA FBW should resume. Repeat once without GS to confirm NAV-only final guidance also hands off. Complete flare, touchdown, braking, and thrust-reverser operation manually.
5. Watch `vdev` in the log while capturing GS. Positive values mean the aircraft is above the path and negative values mean it is below. The error should move decisively toward zero with no sustained lazy offset.

## Other checks

- With guidance modes off, confirm normal AA FBW operation.
- Test `Hdg`, `Alt`, and `Vel` Speed control separately before coupling them.
- Briefly test 2x physics warp in cruise and 4x only if stable; return to 1x for the approach.
- A separate CDI mod will be tested in a later compatibility phase.

Log lines beginning with `[AtmosphereAutopilot NAV/ILS]` report the active plan/leg, modes, commanded and actual track and vertical speed, airspeed, cross-track error, controls, and throttle.

## 0.5.3 visual and indicator checks

- Editable target/approach-speed fields stay white when idle, hovered, and focused. Read-only values stay green and centered vertically and horizontally.
- Check the added gap above section bars, approach-speed rows, and flight-plan row spacing against Kramax.
- For every target button (Bank, Hdg, Pitch, VSpd, Alt, RAlt, Vel, Acc, IAF, FAF, Final), verify a successful Set posts the appropriate yellow confirmation on the main screen. Editing alone must not post a message or apply a value.
- Select NAV: horizontal deviation bar appears above the navball. Select GS: vertical bar appears on its right. Changing to another mode hides the corresponding bar; sections need not be engaged to view deviations.
- Fly right of the planned course: the cyan diamond should move left. Fly above the glidepath: the vertical diamond should move down. Signed labels report aircraft displacement in metres.
- 25 m deviation is half scale, 100 m full scale; beyond 100 m the diamond stays at the edge and turns red. Without a usable leg/path, display `---` without a diamond.
- Change resolution/UI scale or move the navball: bars should remain aligned. Hiding the AtmosphereAutopilot NAV/ILS window or game UI hides the overlay.
- Verify viewing the bars with guidance off does not advance the active waypoint or capture a direct-to origin. Compare the display against Kramax with the same selected leg.

## Indicator dragging (0.5.3)

Left-click and drag either visible deviation bar: both bars move together and retain their offset when switching modes or hiding/showing AtmosphereAutopilot NAV/ILS during the flight. Release outside the original bar to verify capture ends correctly. Hiding the UI mid-drag must release the mouse. The navball and empty space between the bars remain clickable. Repeat with NAV only, GS only, and both selected. Position is retained for this controller instance, not saved across game restarts.
