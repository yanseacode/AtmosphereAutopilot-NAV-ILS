using System;
using System.Globalization;

namespace AtmosphereAutopilotNav.Core
{
    public static class DeviationDisplay
    {
        // Kramax's display scale: 25 m is half scale and 100 m is full scale.
        public static double Deflection(double meters)
        {
            if (!NavigationMath.Finite(meters)) return 0;
            return Math.Sign(meters) * Math.Min(1, Math.Sqrt(Math.Abs(meters)) / 10);
        }

        public static string Label(double meters)
        {
            if (!NavigationMath.Finite(meters)) return "---";
            if (Math.Abs(meters) < 0.5) return "0 m";
            return meters.ToString("+0;-0;0", CultureInfo.InvariantCulture) + " m";
        }
    }
}
