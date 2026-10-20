using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using AtmosphereAutopilotNav.Core;
using CoreWaypoint = AtmosphereAutopilotNav.Core.NavWaypoint;

namespace AtmosphereAutopilotNav.Ksp
{
    public static class NativePlanLoader
    {
        public const int SupportedFormatVersion = 1;

        public static IReadOnlyList<NavPlan> LoadDirectory(string directory, IList<string> errors)
        {
            if (errors == null) throw new ArgumentNullException(nameof(errors));
            var plans = new List<NavPlan>();
            if (!Directory.Exists(directory)) return plans;

            foreach (string file in Directory.GetFiles(directory, "*.cfg", SearchOption.TopDirectoryOnly))
            {
                try
                {
                    ConfigNode root = ConfigNode.Load(file);
                    if (root == null) throw new FormatException("ConfigNode loader returned no data");
                    foreach (ConfigNode node in root.GetNodes("AA_NAV_PLAN"))
                        plans.Add(ParsePlan(node));
                }
                catch (Exception ex)
                {
                    errors.Add(Path.GetFileName(file) + ": " + ex.Message);
                }
            }
            return plans;
        }

        public static NavPlan ParsePlan(ConfigNode node)
        {
            if (node == null) throw new ArgumentNullException(nameof(node));
            int version;
            if (!int.TryParse(Required(node, "formatVersion"), NumberStyles.Integer, CultureInfo.InvariantCulture, out version) ||
                version != SupportedFormatVersion)
                throw new FormatException("Unsupported or missing formatVersion");

            var waypoints = new List<CoreWaypoint>();
            foreach (ConfigNode waypoint in node.GetNodes("WAYPOINT"))
            {
                double? speed = OptionalDouble(waypoint, "speed");
                waypoints.Add(new CoreWaypoint(Required(waypoint, "id"), Point(waypoint), speed,
                    ParseRole(waypoint.GetValue("role")), OptionalBool(waypoint, "vertical", true)));
            }

            var approaches = new List<ApproachDefinition>();
            foreach (ConfigNode approach in node.GetNodes("APPROACH"))
            {
                var threshold = new GeoPoint(Double(approach, "thresholdLatitude"),
                    Double(approach, "thresholdLongitude"), Double(approach, "thresholdAltitude"));
                approaches.Add(new ApproachDefinition(Required(approach, "id"), threshold,
                    Double(approach, "inboundCourse"), Double(approach, "glideAngle"),
                    Double(approach, "interceptDistance"), Double(approach, "handoffHeight")));
            }

            if (waypoints.Count < 2 && approaches.Count == 0)
                throw new FormatException("A plan needs at least two waypoints or one approach");
            return new NavPlan(Required(node, "id"), Required(node, "name"), Required(node, "body"),
                waypoints, approaches, node.GetValue("description"));
        }

        private static WaypointRole ParseRole(string value)
        {
            if (string.IsNullOrWhiteSpace(value)) return WaypointRole.None;
            switch (value.Trim().ToUpperInvariant())
            {
                case "IAF": return WaypointRole.InitialApproachFix;
                case "FAF": return WaypointRole.FinalApproachFix;
                case "RW":
                case "RUNWAY": return WaypointRole.Runway;
                case "STOP": return WaypointRole.Stop;
                default: throw new FormatException("Unknown waypoint role " + value);
            }
        }

        private static GeoPoint Point(ConfigNode node) =>
            new GeoPoint(Double(node, "latitude"), Double(node, "longitude"), Double(node, "altitude"));

        private static string Required(ConfigNode node, string name)
        {
            string value = node.GetValue(name);
            if (string.IsNullOrWhiteSpace(value)) throw new FormatException("Missing " + name);
            return value.Trim();
        }

        private static double Double(ConfigNode node, string name)
        {
            double value;
            if (!double.TryParse(Required(node, name), NumberStyles.Float, CultureInfo.InvariantCulture, out value) ||
                !NavigationMath.Finite(value)) throw new FormatException("Invalid " + name);
            return value;
        }

        private static double? OptionalDouble(ConfigNode node, string name)
        {
            string text = node.GetValue(name);
            if (string.IsNullOrWhiteSpace(text)) return null;
            double value;
            if (!double.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out value) ||
                !NavigationMath.Finite(value)) throw new FormatException("Invalid " + name);
            return value;
        }

        private static bool OptionalBool(ConfigNode node, string name, bool fallback)
        {
            string text = node.GetValue(name);
            if (string.IsNullOrWhiteSpace(text)) return fallback;
            bool value;
            if (!bool.TryParse(text, out value)) throw new FormatException("Invalid " + name);
            return value;
        }
    }
}
