using System;

namespace AtmosphereAutopilotNav.Core
{
    public static class NavigationMath
    {
        private const double DegreesToRadians = Math.PI / 180.0;
        private const double RadiansToDegrees = 180.0 / Math.PI;

        public static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
        public static double Clamp(double value, double minimum, double maximum) => Math.Max(minimum, Math.Min(maximum, value));
        public static double Wrap360(double degrees) => (degrees % 360 + 360) % 360;
        public static double WrapLongitude(double degrees) => Wrap360(degrees + 180) - 180;
        public static double SignedAngle(double targetDegrees, double currentDegrees) => WrapLongitude(targetDegrees - currentDegrees);

        public static double AngularDistanceRadians(GeoPoint from, GeoPoint to)
        {
            double lat1 = from.LatitudeDegrees * DegreesToRadians;
            double lat2 = to.LatitudeDegrees * DegreesToRadians;
            double dLat = lat2 - lat1;
            double dLon = SignedAngle(to.LongitudeDegrees, from.LongitudeDegrees) * DegreesToRadians;
            double sinLat = Math.Sin(dLat / 2);
            double sinLon = Math.Sin(dLon / 2);
            double a = sinLat * sinLat + Math.Cos(lat1) * Math.Cos(lat2) * sinLon * sinLon;
            return 2 * Math.Atan2(Math.Sqrt(a), Math.Sqrt(Math.Max(0, 1 - a)));
        }

        public static double DistanceMeters(GeoPoint from, GeoPoint to, double bodyRadiusMeters)
        {
            ValidateRadius(bodyRadiusMeters);
            return AngularDistanceRadians(from, to) * bodyRadiusMeters;
        }

        public static double InitialBearingDegrees(GeoPoint from, GeoPoint to)
        {
            double lat1 = from.LatitudeDegrees * DegreesToRadians;
            double lat2 = to.LatitudeDegrees * DegreesToRadians;
            double dLon = SignedAngle(to.LongitudeDegrees, from.LongitudeDegrees) * DegreesToRadians;
            double y = Math.Sin(dLon) * Math.Cos(lat2);
            double x = Math.Cos(lat1) * Math.Sin(lat2) - Math.Sin(lat1) * Math.Cos(lat2) * Math.Cos(dLon);
            if (Math.Abs(x) < 1e-14 && Math.Abs(y) < 1e-14) return 0;
            return Wrap360(Math.Atan2(y, x) * RadiansToDegrees);
        }

        public static GeoPoint DestinationPoint(GeoPoint start, double bearingDegrees, double distanceMeters,
            double bodyRadiusMeters)
        {
            ValidateRadius(bodyRadiusMeters);
            if (!Finite(bearingDegrees) || !Finite(distanceMeters) || distanceMeters < 0)
                throw new ArgumentException("Invalid destination input");
            double angularDistance = distanceMeters / bodyRadiusMeters;
            double bearing = bearingDegrees * DegreesToRadians;
            double lat1 = start.LatitudeDegrees * DegreesToRadians;
            double lon1 = start.LongitudeDegrees * DegreesToRadians;
            double lat2 = Math.Asin(Clamp(Math.Sin(lat1) * Math.Cos(angularDistance) +
                Math.Cos(lat1) * Math.Sin(angularDistance) * Math.Cos(bearing), -1, 1));
            double lon2 = lon1 + Math.Atan2(Math.Sin(bearing) * Math.Sin(angularDistance) * Math.Cos(lat1),
                Math.Cos(angularDistance) - Math.Sin(lat1) * Math.Sin(lat2));
            return new GeoPoint(lat2 * RadiansToDegrees, lon2 * RadiansToDegrees, start.AltitudeMeters);
        }

        public static double SignedCrossTrackMeters(GeoPoint legStart, GeoPoint legEnd, GeoPoint position,
            double bodyRadiusMeters)
        {
            ValidateRadius(bodyRadiusMeters);
            double distance13 = AngularDistanceRadians(legStart, position);
            if (distance13 < 1e-12) return 0;
            double bearing13 = InitialBearingDegrees(legStart, position) * DegreesToRadians;
            double bearing12 = InitialBearingDegrees(legStart, legEnd) * DegreesToRadians;
            return Math.Asin(Clamp(Math.Sin(distance13) * Math.Sin(bearing13 - bearing12), -1, 1)) * bodyRadiusMeters;
        }

        public static double AlongTrackMeters(GeoPoint legStart, GeoPoint legEnd, GeoPoint position,
            double bodyRadiusMeters)
        {
            ValidateRadius(bodyRadiusMeters);
            double distance13 = AngularDistanceRadians(legStart, position);
            double bearing13 = InitialBearingDegrees(legStart, position) * DegreesToRadians;
            double bearing12 = InitialBearingDegrees(legStart, legEnd) * DegreesToRadians;
            return Math.Atan2(Math.Sin(distance13) * Math.Cos(bearing13 - bearing12),
                Math.Cos(distance13)) * bodyRadiusMeters;
        }

        public static double InterceptTrackDegrees(double legCourseDegrees, double crossTrackMeters,
            double groundSpeedMetersPerSecond, double interceptTimeSeconds = 10, double maximumInterceptDegrees = 30)
        {
            if (!Finite(legCourseDegrees) || !Finite(crossTrackMeters) || !Finite(groundSpeedMetersPerSecond) || groundSpeedMetersPerSecond <= 0 ||
                !Finite(interceptTimeSeconds) || interceptTimeSeconds <= 0 || !Finite(maximumInterceptDegrees) || maximumInterceptDegrees <= 0 || maximumInterceptDegrees >= 90)
                throw new ArgumentException("Invalid lateral-guidance input");
            double crossVelocity = Clamp(-crossTrackMeters / interceptTimeSeconds,
                -groundSpeedMetersPerSecond * Math.Sin(maximumInterceptDegrees * DegreesToRadians),
                groundSpeedMetersPerSecond * Math.Sin(maximumInterceptDegrees * DegreesToRadians));
            return Wrap360(legCourseDegrees + Math.Asin(crossVelocity / groundSpeedMetersPerSecond) * RadiansToDegrees);
        }

        public static double TurnAnticipationDistanceMeters(double groundSpeedMetersPerSecond,
            double currentCourseDegrees, double nextCourseDegrees, double turnRateDegreesPerSecond = 3)
        {
            if (!Finite(groundSpeedMetersPerSecond) || groundSpeedMetersPerSecond < 0 ||
                !Finite(currentCourseDegrees) || !Finite(nextCourseDegrees) ||
                !Finite(turnRateDegreesPerSecond) || turnRateDegreesPerSecond <= 0)
                throw new ArgumentException("Invalid turn-anticipation input");
            double turnDegrees = Math.Abs(SignedAngle(nextCourseDegrees, currentCourseDegrees));
            return groundSpeedMetersPerSecond * turnDegrees / turnRateDegreesPerSecond;
        }

        public static double GlidepathAltitudeMeters(double thresholdAltitudeMeters, double distanceFromThresholdMeters,
            double glideAngleDegrees)
        {
            if (!Finite(thresholdAltitudeMeters) || !Finite(distanceFromThresholdMeters) || distanceFromThresholdMeters < 0 ||
                !Finite(glideAngleDegrees) || glideAngleDegrees <= 0 || glideAngleDegrees >= 10)
                throw new ArgumentException("Invalid glidepath input");
            return thresholdAltitudeMeters + Math.Tan(glideAngleDegrees * DegreesToRadians) * distanceFromThresholdMeters;
        }

        public static double GlidepathVerticalSpeed(double targetAltitudeMeters, double actualAltitudeMeters,
            double groundSpeedMetersPerSecond, double glideAngleDegrees, double captureTimeSeconds = 8)
        {
            if (!Finite(targetAltitudeMeters) || !Finite(actualAltitudeMeters) || !Finite(groundSpeedMetersPerSecond) || groundSpeedMetersPerSecond <= 0 ||
                !Finite(glideAngleDegrees) || glideAngleDegrees <= 0 || glideAngleDegrees >= 10 || !Finite(captureTimeSeconds) || captureTimeSeconds <= 0)
                throw new ArgumentException("Invalid glideslope-guidance input");
            double pathRate = -groundSpeedMetersPerSecond * Math.Tan(glideAngleDegrees * DegreesToRadians);
            double correction = Clamp((targetAltitudeMeters - actualAltitudeMeters) / captureTimeSeconds,
                -0.1 * groundSpeedMetersPerSecond, 0.1 * groundSpeedMetersPerSecond);
            return Clamp(pathRate + correction, -0.2 * groundSpeedMetersPerSecond, 0.1 * groundSpeedMetersPerSecond);
        }

        public static double AdvanceTransition(double current, double deltaTimeSeconds, double durationSeconds)
        {
            if (!Finite(current) || current < 0 || current > 1 || !Finite(deltaTimeSeconds) || deltaTimeSeconds <= 0 ||
                !Finite(durationSeconds) || durationSeconds <= 0) throw new ArgumentException("Invalid transition input");
            return Clamp(current + deltaTimeSeconds / durationSeconds, 0, 1);
        }

        public static double SmoothTransition(double from, double to, double amount)
        {
            if (!Finite(from) || !Finite(to) || !Finite(amount) || amount < 0 || amount > 1)
                throw new ArgumentException("Invalid transition value");
            double smooth = amount * amount * (3 - 2 * amount);
            return from + (to - from) * smooth;
        }

        public static double SlewAngle(double currentDegrees, double targetDegrees, double maximumRateDegreesPerSecond,
            double deltaTimeSeconds)
        {
            if (!Finite(currentDegrees) || !Finite(targetDegrees) || !Finite(maximumRateDegreesPerSecond) ||
                maximumRateDegreesPerSecond <= 0 || !Finite(deltaTimeSeconds) || deltaTimeSeconds <= 0)
                throw new ArgumentException("Invalid angular slew input");
            double step = maximumRateDegreesPerSecond * deltaTimeSeconds;
            return Wrap360(currentDegrees + Clamp(SignedAngle(targetDegrees, currentDegrees), -step, step));
        }

        public static double MoveTowards(double current, double target, double maximumRatePerSecond, double deltaTimeSeconds)
        {
            if (!Finite(current) || !Finite(target) || !Finite(maximumRatePerSecond) || maximumRatePerSecond <= 0 ||
                !Finite(deltaTimeSeconds) || deltaTimeSeconds <= 0)
                throw new ArgumentException("Invalid linear slew input");
            double step = maximumRatePerSecond * deltaTimeSeconds;
            return current + Clamp(target - current, -step, step);
        }

        public static double LateralAcceleration(double targetTrackDegrees, double actualTrackDegrees,
            double speedMetersPerSecond, double gravityMetersPerSecondSquared)
        {
            if (!Finite(targetTrackDegrees) || !Finite(actualTrackDegrees) || !Finite(speedMetersPerSecond) ||
                speedMetersPerSecond < 1 || !Finite(gravityMetersPerSecondSquared) || gravityMetersPerSecondSquared <= 0)
                throw new ArgumentException("Invalid lateral acceleration input");
            double requested = speedMetersPerSecond * SignedAngle(targetTrackDegrees, actualTrackDegrees) *
                DegreesToRadians / 5.0;
            double limit = gravityMetersPerSecondSquared * Math.Tan(25 * DegreesToRadians);
            return Clamp(requested, -limit, limit);
        }

        public static double RequiredVerticalLift(double targetVerticalSpeed, double actualVerticalSpeed,
            double curvatureAcceleration, double naturalVerticalAcceleration, double liftUpProjection,
            double gravityMetersPerSecondSquared)
        {
            if (!Finite(targetVerticalSpeed) || !Finite(actualVerticalSpeed) || !Finite(curvatureAcceleration) ||
                !Finite(naturalVerticalAcceleration) || !Finite(liftUpProjection) || liftUpProjection < 0.5 ||
                !Finite(gravityMetersPerSecondSquared) || gravityMetersPerSecondSquared <= 0)
                throw new ArgumentException("Vertical guidance requires an upright aircraft");
            double requestedAcceleration = Clamp((targetVerticalSpeed - actualVerticalSpeed) / 5.0,
                -0.25 * gravityMetersPerSecondSquared, 0.25 * gravityMetersPerSecondSquared);
            return Math.Max(0, (requestedAcceleration - curvatureAcceleration - naturalVerticalAcceleration) /
                liftUpProjection);
        }

        private static void ValidateRadius(double bodyRadiusMeters)
        {
            if (!Finite(bodyRadiusMeters) || bodyRadiusMeters <= 0)
                throw new ArgumentOutOfRangeException(nameof(bodyRadiusMeters));
        }
    }
}
