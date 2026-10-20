using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using AtmosphereAutopilotNav.Core;
using CoreWaypoint = AtmosphereAutopilotNav.Core.NavWaypoint;

namespace AtmosphereAutopilotNav.Ksp
{
    // Independent reader for Kramax's public ConfigNode flight-plan interchange format.
    // No Kramax assembly reference is required.
    public static class KramaxPlanLoader
    {
        public const string UserPlanRelativePath = "GameData/KramaxAutoPilot/FlightPlans.cfg";

        public static IReadOnlyList<NavPlan> LoadInstallation(string kspRoot, IList<string> errors)
        {
            if (errors == null) throw new ArgumentNullException(nameof(errors));
            var plans = new List<NavPlan>();
            var index = new Dictionary<string, int>(StringComparer.OrdinalIgnoreCase);
            string directory = Path.Combine(kspRoot, "GameData", "KramaxAutoPilot");
            LoadFile(Path.Combine(directory, "DefaultFlightPlans.cfg"), plans, index, errors);
            LoadFile(Path.Combine(directory, "FlightPlans.cfg"), plans, index, errors);
            return plans;
        }

        private static void LoadFile(string path, List<NavPlan> plans, Dictionary<string, int> index,
            IList<string> errors)
        {
            if (!File.Exists(path)) return;
            try
            {
                ConfigNode root = ConfigNode.Load(path);
                if (root == null) throw new FormatException("ConfigNode loader returned no data");
                foreach (ConfigNode container in root.nodes)
                {
                    if (container.name != "KramaxAutoPilotPlans" &&
                        container.name != "KramaxAutoPilotPlansDefault") continue;
                    foreach (ConfigNode bodyNode in container.nodes)
                    {
                        foreach (ConfigNode planNode in bodyNode.GetNodes("FlightPlan"))
                        {
                            try
                            {
                                NavPlan plan = ParsePlan(planNode, bodyNode.name);
                                string key = plan.Body + "\n" + plan.Name;
                                int existing;
                                if (index.TryGetValue(key, out existing)) plans[existing] = plan;
                                else
                                {
                                    index[key] = plans.Count;
                                    plans.Add(plan);
                                }
                            }
                            catch (Exception ex)
                            {
                                string planName = planNode.GetValue("name") ?? "<unnamed>";
                                errors.Add(Path.GetFileName(path) + " / " + planName + ": " + ex.Message);
                            }
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                errors.Add(Path.GetFileName(path) + ": " + ex.Message);
            }
        }

        public static NavPlan ParsePlan(ConfigNode node, string bodyFallback)
        {
            string name = Required(node, "name");
            string body = node.GetValue("planet");
            if (string.IsNullOrWhiteSpace(body)) body = bodyFallback;
            if (string.IsNullOrWhiteSpace(body)) throw new FormatException("Missing planet");
            var waypoints = new List<CoreWaypoint>();
            ConfigNode waypointContainer = node.GetNode("WayPoints");
            if (waypointContainer != null)
            {
                foreach (ConfigNode waypoint in waypointContainer.GetNodes("WayPoint"))
                {
                    WaypointRole role = WaypointRole.None;
                    if (True(waypoint, "IAF")) role = WaypointRole.InitialApproachFix;
                    if (True(waypoint, "FAF")) role = WaypointRole.FinalApproachFix;
                    if (True(waypoint, "RW")) role = WaypointRole.Runway;
                    if (True(waypoint, "Stop")) role = WaypointRole.Stop;
                    string waypointName = waypoint.GetValue("name");
                    if (string.IsNullOrWhiteSpace(waypointName))
                        waypointName = string.Format(CultureInfo.InvariantCulture, "{0:F2},{1:F2}",
                            LegacyDouble(waypoint, "lat"), LegacyDouble(waypoint, "lon"));
                    waypoints.Add(new CoreWaypoint(waypointName,
                        new GeoPoint(LegacyDouble(waypoint, "lat"), LegacyDouble(waypoint, "lon"),
                            LegacyDouble(waypoint, "alt")),
                        null, role, True(waypoint, "Vertical")));
                }
            }
            if (waypoints.Count == 0) throw new FormatException(name + " has no waypoints");
            string id = "kramax:" + body.Trim() + ":" + name;
            return new NavPlan(id, name, body.Trim(), waypoints, new List<ApproachDefinition>(),
                node.GetValue("description"));
        }

        public static void SaveUserPlan(string kspRoot, NavPlan plan, string previousName)
        {
            if (string.IsNullOrWhiteSpace(kspRoot)) throw new ArgumentException("KSP root is required", nameof(kspRoot));
            if (plan == null) throw new ArgumentNullException(nameof(plan));

            string path = Path.Combine(kspRoot, UserPlanRelativePath.Replace('/', Path.DirectorySeparatorChar));
            Directory.CreateDirectory(Path.GetDirectoryName(path));
            ConfigNode root = File.Exists(path) ? ConfigNode.Load(path) : new ConfigNode();
            if (root == null) root = new ConfigNode();

            ConfigNode container = null;
            foreach (ConfigNode node in root.nodes)
                if (node.name == "KramaxAutoPilotPlans") { container = node; break; }
            if (container == null) container = root.AddNode("KramaxAutoPilotPlans");

            ConfigNode body = container.GetNode(plan.Body) ?? container.AddNode(plan.Body);
            ConfigNode replaced = null;
            foreach (ConfigNode node in body.GetNodes("FlightPlan"))
            {
                string name = node.GetValue("name");
                if (string.Equals(name, previousName, StringComparison.OrdinalIgnoreCase) ||
                    string.Equals(name, plan.Name, StringComparison.OrdinalIgnoreCase))
                {
                    replaced = node;
                    break;
                }
            }
            if (replaced != null) body.RemoveNode(replaced);
            body.AddNode(ToConfigNode(plan));

            string temp = path + ".tmp";
            string backup = path + ".bak";
            root.Save(temp);
            if (File.Exists(path))
            {
                File.Copy(path, backup, true);
                File.Delete(path);
            }
            File.Move(temp, path);
        }

        private static ConfigNode ToConfigNode(NavPlan plan)
        {
            var result = new ConfigNode("FlightPlan");
            var waypoints = result.AddNode("WayPoints");
            foreach (CoreWaypoint waypoint in plan.Waypoints)
            {
                var node = waypoints.AddNode("WayPoint");
                if (waypoint.HasVerticalGuidance) node.AddValue("Vertical", "true");
                if (waypoint.Role == WaypointRole.InitialApproachFix) node.AddValue("IAF", "true");
                if (waypoint.Role == WaypointRole.FinalApproachFix) node.AddValue("FAF", "true");
                if (waypoint.Role == WaypointRole.Runway) node.AddValue("RW", "true");
                if (waypoint.Role == WaypointRole.Stop) node.AddValue("Stop", "true");
                const string coordinateFormat = "0.###############";
                node.AddValue("lat", waypoint.Position.LatitudeDegrees.ToString(coordinateFormat, CultureInfo.InvariantCulture));
                node.AddValue("lon", waypoint.Position.LongitudeDegrees.ToString(coordinateFormat, CultureInfo.InvariantCulture));
                node.AddValue("alt", waypoint.Position.AltitudeMeters.ToString(coordinateFormat, CultureInfo.InvariantCulture));
                node.AddValue("name", waypoint.Id);
            }
            result.AddValue("planet", plan.Body);
            result.AddValue("name", plan.Name);
            result.AddValue("description", plan.Description);
            return result;
        }

        private static bool True(ConfigNode node, string name) =>
            string.Equals(node.GetValue(name), "true", StringComparison.OrdinalIgnoreCase);

        private static string Required(ConfigNode node, string name)
        {
            string value = node.GetValue(name);
            if (string.IsNullOrWhiteSpace(value)) throw new FormatException("Missing " + name);
            return value.Trim();
        }

        private static double LegacyDouble(ConfigNode node, string name)
        {
            double value;
            string text = node.GetValue(name);
            // Kramax's reader uses TryParse and retains zero when a value is malformed.
            // Preserve that behavior so an existing plan loads identically here.
            return double.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out value) &&
                NavigationMath.Finite(value) ? value : 0;
        }
    }
}
