using System;
using System.Collections.Generic;

namespace AtmosphereAutopilotNav.Core
{
    public enum WaypointRole { None, InitialApproachFix, FinalApproachFix, Runway, Stop }

    public struct GeoPoint
    {
        public readonly double LatitudeDegrees;
        public readonly double LongitudeDegrees;
        public readonly double AltitudeMeters;

        public GeoPoint(double latitudeDegrees, double longitudeDegrees, double altitudeMeters = 0)
        {
            if (!NavigationMath.Finite(latitudeDegrees) || latitudeDegrees < -90 || latitudeDegrees > 90)
                throw new ArgumentOutOfRangeException(nameof(latitudeDegrees));
            if (!NavigationMath.Finite(longitudeDegrees))
                throw new ArgumentOutOfRangeException(nameof(longitudeDegrees));
            if (!NavigationMath.Finite(altitudeMeters))
                throw new ArgumentOutOfRangeException(nameof(altitudeMeters));
            LatitudeDegrees = latitudeDegrees;
            LongitudeDegrees = NavigationMath.WrapLongitude(longitudeDegrees);
            AltitudeMeters = altitudeMeters;
        }
    }

    public sealed class NavWaypoint
    {
        public string Id { get; }
        public GeoPoint Position { get; }
        public double? SpeedMetersPerSecond { get; }
        public WaypointRole Role { get; }
        public bool HasVerticalGuidance { get; }

        public NavWaypoint(string id, GeoPoint position, double? speedMetersPerSecond = null,
            WaypointRole role = WaypointRole.None, bool hasVerticalGuidance = true)
        {
            if (string.IsNullOrWhiteSpace(id)) throw new ArgumentException("Waypoint id is required", nameof(id));
            if (speedMetersPerSecond.HasValue && (!NavigationMath.Finite(speedMetersPerSecond.Value) || speedMetersPerSecond.Value <= 0))
                throw new ArgumentOutOfRangeException(nameof(speedMetersPerSecond));
            Id = id.Trim();
            Position = position;
            SpeedMetersPerSecond = speedMetersPerSecond;
            Role = role;
            HasVerticalGuidance = hasVerticalGuidance;
        }
    }

    public sealed class ApproachDefinition
    {
        public string Id { get; }
        public GeoPoint Threshold { get; }
        public double InboundCourseDegrees { get; }
        public double GlideAngleDegrees { get; }
        public double InterceptDistanceMeters { get; }
        public double HandoffHeightMeters { get; }

        public ApproachDefinition(string id, GeoPoint threshold, double inboundCourseDegrees,
            double glideAngleDegrees, double interceptDistanceMeters, double handoffHeightMeters)
        {
            if (string.IsNullOrWhiteSpace(id)) throw new ArgumentException("Approach id is required", nameof(id));
            if (!NavigationMath.Finite(inboundCourseDegrees)) throw new ArgumentOutOfRangeException(nameof(inboundCourseDegrees));
            if (!NavigationMath.Finite(glideAngleDegrees) || glideAngleDegrees <= 0 || glideAngleDegrees >= 10)
                throw new ArgumentOutOfRangeException(nameof(glideAngleDegrees));
            if (!NavigationMath.Finite(interceptDistanceMeters) || interceptDistanceMeters <= 0)
                throw new ArgumentOutOfRangeException(nameof(interceptDistanceMeters));
            if (!NavigationMath.Finite(handoffHeightMeters) || handoffHeightMeters < 0)
                throw new ArgumentOutOfRangeException(nameof(handoffHeightMeters));
            Id = id.Trim();
            Threshold = threshold;
            InboundCourseDegrees = NavigationMath.Wrap360(inboundCourseDegrees);
            GlideAngleDegrees = glideAngleDegrees;
            InterceptDistanceMeters = interceptDistanceMeters;
            HandoffHeightMeters = handoffHeightMeters;
        }
    }

    public sealed class NavPlan
    {
        public string Id { get; }
        public string Name { get; }
        public string Body { get; }
        public string Description { get; }
        public IReadOnlyList<NavWaypoint> Waypoints { get; }
        public IReadOnlyList<ApproachDefinition> Approaches { get; }

        public NavPlan(string id, string name, string body, IReadOnlyList<NavWaypoint> waypoints,
            IReadOnlyList<ApproachDefinition> approaches, string description = null)
        {
            if (string.IsNullOrWhiteSpace(id)) throw new ArgumentException("Plan id is required", nameof(id));
            if (string.IsNullOrWhiteSpace(name)) throw new ArgumentException("Plan name is required", nameof(name));
            if (string.IsNullOrWhiteSpace(body)) throw new ArgumentException("Body is required", nameof(body));
            Id = id.Trim();
            Name = name.Trim();
            Body = body.Trim();
            Description = string.IsNullOrWhiteSpace(description) ? Name : description.Trim();
            Waypoints = waypoints ?? throw new ArgumentNullException(nameof(waypoints));
            Approaches = approaches ?? throw new ArgumentNullException(nameof(approaches));
        }
    }
}
