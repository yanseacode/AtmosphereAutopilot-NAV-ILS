using System;
using System.Collections.Generic;
using AtmosphereAutopilotNav.Core;

internal static class NavigationCoreTests
{
    private static int checks;

    private static void Check(bool condition, string name)
    {
        checks++;
        if (!condition) throw new Exception("FAILED: " + name);
    }

    private static void Near(double actual, double expected, double tolerance, string name) =>
        Check(Math.Abs(actual - expected) <= tolerance, name + " expected " + expected + " got " + actual);

    public static int Main()
    {
        const double kerbinRadius = 600000;
        var origin = new GeoPoint(0, 0);
        var east = new GeoPoint(0, 1);
        var north = new GeoPoint(1, 0);
        Near(NavigationMath.InitialBearingDegrees(origin, east), 90, 1e-9, "east bearing");
        Near(NavigationMath.InitialBearingDegrees(origin, north), 0, 1e-9, "north bearing");
        Near(NavigationMath.DistanceMeters(origin, east, kerbinRadius), Math.PI / 180 * kerbinRadius, 0.01, "one degree distance");

        double rightOfNorth = NavigationMath.SignedCrossTrackMeters(origin, north, new GeoPoint(0.5, 0.1), kerbinRadius);
        double leftOfNorth = NavigationMath.SignedCrossTrackMeters(origin, north, new GeoPoint(0.5, -0.1), kerbinRadius);
        Check(rightOfNorth > 0, "right-of-course sign");
        Check(leftOfNorth < 0, "left-of-course sign");
        Near(NavigationMath.AlongTrackMeters(origin, north, new GeoPoint(0.5, 0), kerbinRadius),
            0.5 * Math.PI / 180 * kerbinRadius, 0.01, "along-track midpoint");
        Check(NavigationMath.AlongTrackMeters(origin, north, new GeoPoint(-0.5, 0), kerbinRadius) < 0,
            "behind-start along-track sign");
        GeoPoint destination = NavigationMath.DestinationPoint(origin, 90, Math.PI / 180 * kerbinRadius, kerbinRadius);
        Near(destination.LatitudeDegrees, 0, 1e-9, "destination latitude");
        Near(destination.LongitudeDegrees, 1, 1e-9, "destination longitude");
        Check(NavigationMath.SignedAngle(NavigationMath.InterceptTrackDegrees(0, rightOfNorth, 100), 0) < 0,
            "right-of-course intercept turns left");
        Check(NavigationMath.SignedAngle(NavigationMath.InterceptTrackDegrees(0, leftOfNorth, 100), 0) > 0,
            "left-of-course intercept turns right");

        Near(NavigationMath.GlidepathAltitudeMeters(70, 10000, 3), 70 + Math.Tan(3 * Math.PI / 180) * 10000, 1e-9, "glidepath altitude");
        Check(NavigationMath.GlidepathVerticalSpeed(600, 600, 100, 3) < 0, "on-path glideslope descends");
        Check(NavigationMath.GlidepathVerticalSpeed(600, 500, 100, 3) > NavigationMath.GlidepathVerticalSpeed(600, 700, 100, 3), "glideslope correction direction");

        Near(NavigationMath.AdvanceTransition(0, 0.5, 2.5), 0.2, 1e-12, "transition rate");
        Near(NavigationMath.SmoothTransition(10, 20, 0), 10, 1e-12, "transition start");
        Near(NavigationMath.SmoothTransition(10, 20, 1), 20, 1e-12, "transition end");
        Near(NavigationMath.SmoothTransition(10, 20, 0.5), 15, 1e-12, "transition midpoint");
        Near(NavigationMath.SlewAngle(359, 10, 3, 1), 2, 1e-12, "heading slew across north");
        Near(NavigationMath.SlewAngle(1, 350, 3, 1), 358, 1e-12, "heading slew left across north");
        Near(NavigationMath.MoveTowards(-5, 5, 2, 1.5), -2, 1e-12, "linear slew");
        Check(NavigationMath.LateralAcceleration(20, 0, 100, 9.81) > 0, "right turn acceleration sign");
        Check(NavigationMath.LateralAcceleration(340, 0, 100, 9.81) < 0, "left turn acceleration sign");
        Check(NavigationMath.RequiredVerticalLift(5, 0, 0, -9.81, 1, 9.81) > 9.81,
            "climb requires additional lift");

        var waypoint = new NavWaypoint("FIX", new GeoPoint(-0.1, -74.5, 1500), 120);
        Check(waypoint.Id == "FIX" && waypoint.SpeedMetersPerSecond == 120, "waypoint model");
        var approach = new ApproachDefinition("RWY09", new GeoPoint(-0.05, -74.72, 70), 90, 3, 12000, 150);
        Check(approach.InboundCourseDegrees == 90 && approach.HandoffHeightMeters == 150, "approach model");

        var plan = new NavPlan("test", "Test", "Kerbin", new List<NavWaypoint>
        {
            new NavWaypoint("A", new GeoPoint(0, 0, 1000), 100),
            new NavWaypoint("B", new GeoPoint(0, 1, 1200), 110),
            new NavWaypoint("C", new GeoPoint(1, 1, 1400), 120)
        }, new List<ApproachDefinition> { approach });
        var navigator = new PlanNavigator();
        navigator.SetPlan(plan);
        LegGuidance direct = navigator.Update(new GeoPoint(-0.1, -0.1, 1000), 100, kerbinRadius, 100);
        Check(direct.TargetIndex == 0 && direct.FromId == "Present" && direct.ToId == "A",
            "new plan directs to first waypoint");
        Check(navigator.ActivateLeg(1), "activate planned leg");
        LegGuidance leg = navigator.Update(new GeoPoint(0.1, 0.5, 1100), 100, kerbinRadius, 100);
        Check(leg.TargetIndex == 1 && leg.FromId == "Present" && leg.ToId == "B",
            "ordinary direct-to starts at present position");
        leg = navigator.Update(new GeoPoint(0.05, 0.75, 1150), 100, kerbinRadius, 100);
        Check(leg.TargetIndex == 1, "leg becomes valid before sequencing");
        leg = navigator.Update(new GeoPoint(0, 1.01, 1200), 100, kerbinRadius, 100);
        Check(leg.TargetIndex == 2 && leg.FromId == "B" && leg.ToId == "C", "passed fix sequences leg");
        Check(navigator.PreviousLeg() && navigator.TargetIndex == 1, "manual previous leg");
        Check(navigator.NextLeg() && navigator.TargetIndex == 2, "manual next leg");
        Check(navigator.ActivateLeg(0) && navigator.TargetIndex == 0, "direct-to first waypoint selection");

        var approachPlan = new NavPlan("approach", "Approach", "Kerbin", new List<NavWaypoint>
        {
            new NavWaypoint("IAF", new GeoPoint(0, 0, 1000), null, WaypointRole.InitialApproachFix, true),
            new NavWaypoint("FAF", new GeoPoint(0, 1, 600), null, WaypointRole.FinalApproachFix, true),
            new NavWaypoint("RW", new GeoPoint(0, 2, 70), null, WaypointRole.Runway, true)
        }, new List<ApproachDefinition>());
        navigator.SetPlan(approachPlan);
        Check(navigator.ActivateLeg(1), "activate FAF leg");
        LegGuidance fafLeg = navigator.Update(new GeoPoint(0.2, 0.5, 800), 100, kerbinRadius, 100);
        Check(fafLeg.FromId == "IAF" && fafLeg.ToId == "FAF",
            "FAF direct-to preserves published inbound leg");
        Check(fafLeg.HasVerticalGuidance, "vertical Kramax leg available");
        Near(fafLeg.PathAltitudeMeters, 800, 1, "vertical path interpolation");
        fafLeg = navigator.Update(new GeoPoint(0, 0.995, 605), 100, kerbinRadius, 750);
        Check(fafLeg.TargetIndex == 1, "FAF leg ignores early capture radius");
        Check(navigator.ActivateLeg(2), "activate runway leg");
        LegGuidance runwayLeg = navigator.Update(new GeoPoint(0.1, 1.5, 335), 100, kerbinRadius, 100);
        Check(runwayLeg.FromId == "FAF" && runwayLeg.ToId == "RW",
            "runway direct-to preserves final approach leg");

        navigator.SetPlan(approachPlan);
        LegGuidance beyondPlan = navigator.Update(new GeoPoint(0, 3, 70), 100, kerbinRadius, 100);
        Check(beyondPlan.TargetIndex == 0, "unflown plan cannot cascade through waypoints");

        GeoPoint approachStart = NavigationMath.DestinationPoint(approach.Threshold, 270, 6000, kerbinRadius);
        double onPathAltitude = NavigationMath.GlidepathAltitudeMeters(approach.Threshold.AltitudeMeters, 6000, 3);
        ApproachGuidance approachGuidance = PlanNavigator.Approach(approach, approachStart, onPathAltitude, 100, kerbinRadius);
        Near(approachGuidance.CrossTrackMeters, 0, 0.1, "localizer centerline");
        Near(approachGuidance.DistanceToThresholdMeters, 6000, 0.1, "distance to threshold");
        Near(approachGuidance.GlidepathAltitudeMeters, onPathAltitude, 0.01, "approach path altitude");
        Check(approachGuidance.GlidepathVerticalSpeed < 0, "approach descends");

        bool rejected = false;
        try { new GeoPoint(91, 0); } catch (ArgumentOutOfRangeException) { rejected = true; }
        Check(rejected, "invalid latitude rejected");
        rejected = false;
        try { new ApproachDefinition("bad", origin, 0, 0, 1000, 100); } catch (ArgumentOutOfRangeException) { rejected = true; }
        Check(rejected, "invalid glide angle rejected");

        Near(DeviationDisplay.Deflection(0), 0, 1e-9, "indicator centered on course");
        Near(DeviationDisplay.Deflection(25), 0.5, 1e-9, "indicator half scale at 25 m");
        Near(DeviationDisplay.Deflection(-100), -1, 1e-9, "indicator negative full scale");
        Near(DeviationDisplay.Deflection(400), 1, 1e-9, "indicator clamps beyond full scale");
        Check(DeviationDisplay.Label(5) == "+5 m" && DeviationDisplay.Label(-5) == "-5 m",
            "indicator signed meter labels");
        Check(DeviationDisplay.Label(0.49) == "0 m" && DeviationDisplay.Label(double.NaN) == "---",
            "indicator neutral and unavailable labels");
        navigator.SetPlan(approachPlan);
        navigator.ActivateLeg(1);
        var preview = navigator.Preview(new GeoPoint(0.2, 0.5, 850), 100, kerbinRadius);
        Near(preview.PathAltitudeMeters, 800, 1, "display preview interpolates GS path");
        Check(preview.CrossTrackMeters != 0, "display preview exposes cross-track deviation");
        navigator.Preview(new GeoPoint(0, 1.001, 600), 100, kerbinRadius);
        Check(navigator.TargetIndex == 1, "drawing beyond fix cannot sequence live leg");
        var afterPreview = navigator.Update(new GeoPoint(0, 1.001, 600), 100, kerbinRadius);
        Check(afterPreview.TargetIndex == 1, "drawing cannot mark live leg as previously flown");
        navigator.SetPlan(approachPlan);
        navigator.Preview(new GeoPoint(1, 1, 700), 100, kerbinRadius);
        var freshOrigin = navigator.Update(new GeoPoint(2, 2, 900), 100, kerbinRadius);
        Near(freshOrigin.CrossTrackMeters, 0, 0.001, "drawing cannot capture live direct-to origin");

        Console.WriteLine("PASS: " + checks + " Navigation core checks");
        return 0;
    }
}
