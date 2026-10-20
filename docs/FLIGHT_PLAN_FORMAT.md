# Native flight-plan format

Navigation uses KSP `ConfigNode` files under `GameData/AtmosphereAutopilotNav/FlightPlans`. The format is versioned so it can evolve alongside Kramax-compatible plan loading.

```text
AA_NAV_PLAN
{
    formatVersion = 1
    id = example-plan
    name = Example Plan
    description = Example route and approach
    body = Kerbin

    WAYPOINT
    {
        id = FIX01
        latitude = -0.10
        longitude = -74.50
        altitude = 1500
        speed = 120
        role = IAF
    }

    APPROACH
    {
        id = RWY09
        thresholdLatitude = -0.05
        thresholdLongitude = -74.72
        thresholdAltitude = 70
        inboundCourse = 90
        glideAngle = 3
        interceptDistance = 12000
        handoffHeight = 150
    }
}
```

Angles are degrees, distances and altitudes are metres, and speeds are metres per second. Optional waypoint roles are `IAF`, `FAF`, `RW`, and `STOP`. Coordinates and runway data included in a release must be independently generated and accompanied by provenance notes.
