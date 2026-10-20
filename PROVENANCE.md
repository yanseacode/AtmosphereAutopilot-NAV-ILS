# Provenance record

## Code

The `AtmosphereAutopilotNav` source was written for this project as an independent AtmosphereAutopilot extension. It uses public KSP and AtmosphereAutopilot APIs. AtmosphereAutopilot is an external GPLv3-or-later dependency and is not copied into this source tree.

The local `NavIlsIntegration` prototype demonstrated desired runtime behavior but is not a source dependency. The reader independently converts the installed Kramax ConfigNode flight-plan format into Navigation's internal model and does not reference the Kramax assembly.

## Navigation formulas

The core uses standard spherical-navigation relationships: haversine angular distance, initial great-circle bearing, signed cross-track distance, along-track distance, bounded intercept angle, and a geometric constant-angle glidepath. Formula implementations are independently written and covered by deterministic tests.

## Airport and runway data

`KSC-Development.cfg` is test data, not a final airport database. Its RWY09 reference point comes from a KSP-generated prelaunch vessel record at the stock Runway in `saves/sandbox/LaunchQSave_001_001-00_53_33.sfs` (`lat`, `lon`, and `alt` at lines 12810-12812). The circuit points, final approach fix, and opposite threshold estimate were generated from that point with this project's spherical destination formula using Kerbin's 600,000 m radius. The observed stock runway courses are represented as 090/270. This data was not taken from a third-party flight-plan file and must be remeasured before a stable release.

Kramax flight plans are used in development from the user's installed copy. They are not currently copied into this package. Written redistribution and modification permission plus attribution requirements must be recorded here before they are bundled.

Before release, every generated or permitted dataset must record:

- its factual source;
- the extraction or measurement method;
- the generation date and script version;
- any manual corrections;
- confirmation that no third-party flight-plan file was copied.


## Deviation display (0.5.2)

The overlay is an independently written renderer using local Kramax CDI.cs as a behavioral reference: navball-relative geometry, signed square-root scale (100 m full scale), colors, ticks and marker dimensions. No Kramax code, textures, or assembly are bundled. Plan preview uses Navigation's own geometry and cannot mutate the live navigator.
