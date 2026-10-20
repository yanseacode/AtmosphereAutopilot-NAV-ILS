using System;

namespace AtmosphereAutopilotNav.Core
{
    public struct LegGuidance
    {
        public readonly int TargetIndex;
        public readonly string FromId;
        public readonly string ToId;
        public readonly double CourseDegrees;
        public readonly double DesiredTrackDegrees;
        public readonly double CrossTrackMeters;
        public readonly double DistanceToTargetMeters;
        public readonly double TargetAltitudeMeters;
        public readonly double? TargetSpeedMetersPerSecond;
        public readonly bool HasVerticalGuidance;
        public readonly double PathAltitudeMeters;
        public readonly double PathSlope;

        public LegGuidance(int targetIndex, string fromId, string toId, double courseDegrees,
            double desiredTrackDegrees, double crossTrackMeters, double distanceToTargetMeters,
            double targetAltitudeMeters, double? targetSpeedMetersPerSecond,
            bool hasVerticalGuidance, double pathAltitudeMeters, double pathSlope)
        {
            TargetIndex = targetIndex;
            FromId = fromId;
            ToId = toId;
            CourseDegrees = courseDegrees;
            DesiredTrackDegrees = desiredTrackDegrees;
            CrossTrackMeters = crossTrackMeters;
            DistanceToTargetMeters = distanceToTargetMeters;
            TargetAltitudeMeters = targetAltitudeMeters;
            TargetSpeedMetersPerSecond = targetSpeedMetersPerSecond;
            HasVerticalGuidance = hasVerticalGuidance;
            PathAltitudeMeters = pathAltitudeMeters;
            PathSlope = pathSlope;
        }
    }

    public struct ApproachGuidance
    {
        public readonly string ApproachId;
        public readonly double DesiredTrackDegrees;
        public readonly double CrossTrackMeters;
        public readonly double DistanceToThresholdMeters;
        public readonly double GlidepathAltitudeMeters;
        public readonly double GlidepathVerticalSpeed;
        public readonly double HandoffHeightMeters;
        public readonly bool BeforeThreshold;

        public ApproachGuidance(string approachId, double desiredTrackDegrees, double crossTrackMeters,
            double distanceToThresholdMeters, double glidepathAltitudeMeters, double glidepathVerticalSpeed,
            double handoffHeightMeters, bool beforeThreshold)
        {
            ApproachId = approachId;
            DesiredTrackDegrees = desiredTrackDegrees;
            CrossTrackMeters = crossTrackMeters;
            DistanceToThresholdMeters = distanceToThresholdMeters;
            GlidepathAltitudeMeters = glidepathAltitudeMeters;
            GlidepathVerticalSpeed = glidepathVerticalSpeed;
            HandoffHeightMeters = handoffHeightMeters;
            BeforeThreshold = beforeThreshold;
        }
    }

    public sealed class PlanNavigator
    {
        private GeoPoint? directStart;
        private bool usePlannedPrevious;
        private bool legWasValid;
        public NavPlan Plan { get; private set; }
        public int TargetIndex { get; private set; }
        public bool Complete => Plan == null || Plan.Waypoints.Count == 0 || TargetIndex >= Plan.Waypoints.Count;

        public void SetPlan(NavPlan plan)
        {
            Plan = plan ?? throw new ArgumentNullException(nameof(plan));
            TargetIndex = plan.Waypoints.Count > 0 ? 0 : plan.Waypoints.Count;
            directStart = null;
            usePlannedPrevious = false;
            legWasValid = false;
        }

        public void Clear()
        {
            Plan = null;
            TargetIndex = 0;
            directStart = null;
            usePlannedPrevious = false;
            legWasValid = false;
        }

        public bool PreviousLeg()
        {
            if (Plan == null || Plan.Waypoints.Count == 0 || TargetIndex <= 0) return false;
            TargetIndex--;
            directStart = null;
            usePlannedPrevious = true;
            legWasValid = false;
            return true;
        }

        public bool NextLeg()
        {
            if (Plan == null || TargetIndex >= Plan.Waypoints.Count) return false;
            TargetIndex++;
            directStart = null;
            usePlannedPrevious = true;
            legWasValid = false;
            return !Complete;
        }

        public bool ActivateLeg(int targetIndex)
        {
            if (Plan == null || targetIndex < 0 || targetIndex >= Plan.Waypoints.Count) return false;
            TargetIndex = targetIndex;
            directStart = null;
            WaypointRole role = Plan.Waypoints[targetIndex].Role;
            usePlannedPrevious = targetIndex > 0 &&
                (role == WaypointRole.FinalApproachFix || role == WaypointRole.Runway);
            legWasValid = false;
            return true;
        }

        // Rendering must never capture a direct-to origin or advance the live plan.
        public LegGuidance Preview(GeoPoint position, double speed, double radius)
        {
            var copy = new PlanNavigator { Plan = Plan, TargetIndex = TargetIndex,
                directStart = directStart, usePlannedPrevious = usePlannedPrevious, legWasValid = legWasValid };
            return copy.Update(position, speed, radius, 750, false);
        }

        public LegGuidance Update(GeoPoint position, double groundSpeedMetersPerSecond, double bodyRadiusMeters,
            double captureRadiusMeters = 750, bool advance = true)
        {
            if (Complete) throw new InvalidOperationException("No active flight-plan leg");
            if (!NavigationMath.Finite(captureRadiusMeters) || captureRadiusMeters <= 0)
                throw new ArgumentOutOfRangeException(nameof(captureRadiusMeters));

            while (!Complete)
            {
                NavWaypoint to = Plan.Waypoints[TargetIndex];
                GeoPoint fromPosition;
                string fromId;
                bool plannedLeg = TargetIndex > 0 && usePlannedPrevious;
                if (!plannedLeg)
                {
                    if (!directStart.HasValue) directStart = position;
                    fromPosition = directStart.Value;
                    fromId = "Present";
                }
                else
                {
                    fromPosition = Plan.Waypoints[TargetIndex - 1].Position;
                    fromId = Plan.Waypoints[TargetIndex - 1].Id;
                }
                double legLength = NavigationMath.DistanceMeters(fromPosition, to.Position, bodyRadiusMeters);
                double alongTrack = NavigationMath.AlongTrackMeters(fromPosition, to.Position, position, bodyRadiusMeters);
                double distance = NavigationMath.DistanceMeters(position, to.Position, bodyRadiusMeters);
                if (alongTrack > 0 && alongTrack < legLength) legWasValid = true;
                bool protectedApproachLeg = to.Role == WaypointRole.FinalApproachFix ||
                    to.Role == WaypointRole.Runway || to.Role == WaypointRole.Stop;
                if (advance && legWasValid && TargetIndex < Plan.Waypoints.Count - 1 &&
                    (alongTrack >= legLength - 10 || (!protectedApproachLeg && distance <= captureRadiusMeters)))
                {
                    TargetIndex++;
                    directStart = null;
                    usePlannedPrevious = true;
                    legWasValid = false;
                    continue;
                }

                double course = NavigationMath.InitialBearingDegrees(fromPosition, to.Position);
                double crossTrack = NavigationMath.SignedCrossTrackMeters(fromPosition, to.Position, position, bodyRadiusMeters);
                double desiredTrack = NavigationMath.InterceptTrackDegrees(course, crossTrack, groundSpeedMetersPerSecond);
                bool fromHasVertical = !plannedLeg || Plan.Waypoints[TargetIndex - 1].HasVerticalGuidance;
                bool hasVertical = to.HasVerticalGuidance && fromHasVertical && legLength > 0;
                double fraction = NavigationMath.Clamp(alongTrack / Math.Max(1, legLength), 0, 1);
                double pathAltitude = hasVertical
                    ? fromPosition.AltitudeMeters + (to.Position.AltitudeMeters - fromPosition.AltitudeMeters) * fraction
                    : double.NaN;
                double pathSlope = hasVertical
                    ? (to.Position.AltitudeMeters - fromPosition.AltitudeMeters) / legLength
                    : double.NaN;
                return new LegGuidance(TargetIndex, fromId, to.Id, course, desiredTrack, crossTrack, distance,
                    to.Position.AltitudeMeters, to.SpeedMetersPerSecond, hasVertical, pathAltitude, pathSlope);
            }
            throw new InvalidOperationException("Flight plan is complete");
        }

        public static ApproachGuidance Approach(ApproachDefinition approach, GeoPoint position,
            double altitudeMeters, double groundSpeedMetersPerSecond, double bodyRadiusMeters)
        {
            if (approach == null) throw new ArgumentNullException(nameof(approach));
            GeoPoint localizerStart = NavigationMath.DestinationPoint(approach.Threshold,
                NavigationMath.Wrap360(approach.InboundCourseDegrees + 180), approach.InterceptDistanceMeters, bodyRadiusMeters);
            double crossTrack = NavigationMath.SignedCrossTrackMeters(localizerStart, approach.Threshold, position, bodyRadiusMeters);
            double alongTrack = NavigationMath.AlongTrackMeters(localizerStart, approach.Threshold, position, bodyRadiusMeters);
            double distanceToThreshold = approach.InterceptDistanceMeters - alongTrack;
            bool beforeThreshold = distanceToThreshold >= 0;
            double pathDistance = Math.Max(0, distanceToThreshold);
            double pathAltitude = NavigationMath.GlidepathAltitudeMeters(approach.Threshold.AltitudeMeters,
                pathDistance, approach.GlideAngleDegrees);
            double verticalSpeed = NavigationMath.GlidepathVerticalSpeed(pathAltitude, altitudeMeters,
                groundSpeedMetersPerSecond, approach.GlideAngleDegrees);
            double desiredTrack = NavigationMath.InterceptTrackDegrees(approach.InboundCourseDegrees,
                crossTrack, groundSpeedMetersPerSecond, 12, 20);
            return new ApproachGuidance(approach.Id, desiredTrack, crossTrack, distanceToThreshold,
                pathAltitude, verticalSpeed, approach.HandoffHeightMeters, beforeThreshold);
        }
    }
}
