using System;
using System.Collections.Generic;
using System.Globalization;
using AtmosphereAutopilot;
using AtmosphereAutopilotNav.Core;
using AtmosphereAutopilotNav.Ksp;
using UnityEngine;
using CoreWaypoint = AtmosphereAutopilotNav.Core.NavWaypoint;
namespace AtmosphereAutopilotNav.Plugin
{
    public enum LateralGuidanceMode { Off, Bank, Heading, Nav }
    public enum VerticalGuidanceMode { Off, Pitch, VerticalSpeed, Altitude, RadarAltitude, Glideslope }
    public enum ThrottleGuidanceMode { Velocity, Acceleration, Landing }
    // AA discovers this sealed high-level controller. AA remains the sole control owner.
    public sealed class NavApproachController : StateController
    {
        private DirectorController director;
        private FlightModel model;
        private ProgradeThrustController thrust;
        private StandardFlyByWire flyByWire;
        private PitchAngularVelocityController pitch;
        private RollAngularVelocityController roll;
        private YawAngularVelocityController yaw;
        private PitchAoAController pitchAoA;
        private SideslipController sideslip;
        private IReadOnlyList<NavPlan> plans = new List<NavPlan>();
        private readonly PlanNavigator navigator = new PlanNavigator();
        private readonly List<string> loadErrors = new List<string>();
        private int planIndex;
        private LateralGuidanceMode lateralMode;
        private VerticalGuidanceMode verticalMode;
        private LateralGuidanceMode selectedLateralMode = LateralGuidanceMode.Heading;
        private VerticalGuidanceMode selectedVerticalMode = VerticalGuidanceMode.VerticalSpeed;
        private ThrottleGuidanceMode selectedThrottleMode = ThrottleGuidanceMode.Velocity;
        private bool speedEnabled;
        private bool lateralWasActive;
        private bool verticalWasActive;
        private bool approachWarningShown;
        private bool savedThrustSettings;
        private bool savedUsePid;
        private bool savedUseBrakes;
        private bool savedPitchModeration;
        private bool savedYawModeration;
        private double bankDegrees;
        private double headingDegrees;
        private double pitchDegrees;
        private double verticalSpeedMetersPerSecond;
        private double altitudeMeters;
        private double radarAltitudeMeters = 500;
        private double speedMetersPerSecond = 100;
        private double commandedSpeedMetersPerSecond = 100;
        private double accelerationMetersPerSecondSquared;
        private double accelerationSpeedSetpoint;
        private bool accelerationSetpointInitialized;
        private double iafSpeedMetersPerSecond = 200;
        private double fafSpeedMetersPerSecond = 175;
        private double finalSpeedMetersPerSecond = 100;
        private double commandedTrack;
        private double commandedVerticalSpeed;
        private string bankText = "0";
        private string headingText = "90";
        private string altitudeText = "1500";
        private string speedText = "100";
        private string pitchText = "0";
        private string verticalSpeedText = "0";
        private string radarAltitudeText = "500";
        private string accelerationText = "0";
        private string iafSpeedText = "200";
        private string fafSpeedText = "175";
        private string finalSpeedText = "100";
        private string status = "FBW standby";
        private LegGuidance lastLeg;
        private bool hasLeg;
        private double lastTelemetryTime;
        private bool flightPlanExpanded = true;
        private bool planManagerShown;
        private Rect planManagerRect = new Rect(740, 180, 315, 450);
        private Vector2 planScroll;
        private string planNameText = "";
        private string planDescriptionText = "";
        private string loadedPlanName = "";
        private bool rollModesVisible = true;
        private bool verticalModesVisible = true;
        private bool throttleModesVisible = true;
        private GUIStyle compactValueStyle;
        private GUIStyle compactLabelStyle;
        private GUIStyle compactButtonStyle;
        private GUIStyle incrementStyle;
        private float separatorDragDistance;
        private int layoutSignature = -1;
        private bool approachSpeedsExpanded;
        private GUIStyle sectionStyle;
        private GUIStyle activeSectionStyle;
        private GUIStyle modeStyle;
        private GUIStyle selectedModeStyle;
        private GUIStyle valueStyle;
        private GUIStyle greenLabelStyle;
        private readonly DeviationOverlay deviationOverlay = new DeviationOverlay();
        public NavApproachController(Vessel vessel)
            : base(vessel, "NAV / ILS", 88439961) { }
        public override void InitializeDependencies(Dictionary<Type, AutopilotModule> modules)
        {
            director = (DirectorController)modules[typeof(DirectorController)];
            model = (FlightModel)modules[typeof(FlightModel)];
            thrust = (ProgradeThrustController)modules[typeof(ProgradeThrustController)];
            flyByWire = (StandardFlyByWire)modules[typeof(StandardFlyByWire)];
            pitch = (PitchAngularVelocityController)modules[typeof(PitchAngularVelocityController)];
            roll = (RollAngularVelocityController)modules[typeof(RollAngularVelocityController)];
            yaw = (YawAngularVelocityController)modules[typeof(YawAngularVelocityController)];
            pitchAoA = (PitchAoAController)modules[typeof(PitchAoAController)];
            sideslip = (SideslipController)modules[typeof(SideslipController)];
        }
        private void ApplyIndependentModes(FlightCtrlState state, double trackDegrees, double verticalSpeed,
            double speed, Vector3d up, double actualVerticalSpeed, bool bankMode, bool pitchMode)
        {
            if (lateralMode == LateralGuidanceMode.Off)
                ApplyManualRoll(state);
            else if (bankMode)
                ApplyBankRoll(state, bankDegrees, up);
            else
                ApplyTrackRoll(state, trackDegrees, speed, up);
            if (verticalMode == VerticalGuidanceMode.Off)
                ApplyManualPitch(state);
            else if (pitchMode)
                ApplyPitchHold(state, pitchDegrees);
            else
                ApplyVerticalPitch(state, verticalSpeed, speed, up, actualVerticalSpeed);
            if (lateralMode == LateralGuidanceMode.Off) ApplyManualYaw(state);
            else ApplyGuidedYaw(state);
        }
        private void ApplyLateralOnly(FlightCtrlState state, double trackDegrees, double speed, Vector3d up)
        {
            ApplyManualPitch(state);
            ApplyTrackRoll(state, trackDegrees, speed, up);
            ApplyGuidedYaw(state);
        }
        private void ApplyTrackRoll(FlightCtrlState state, double trackDegrees, double speed, Vector3d up)
        {
            ControlUtils.neutralize_user_input(state, ROLL);
            Vector3d right = Vector3d.Cross(up, model.surface_v).normalized;
            if (right.sqrMagnitude < 0.5) throw new InvalidOperationException("No horizontal velocity for NAV");
            double lateral = NavigationMath.LateralAcceleration(trackDegrees, CurrentTrackDegrees(), speed,
                model.gravity_acc.magnitude);
            Vector3d lift = -model.gravity_acc - model.noninert_acc + right * lateral;
            Vector3d normalLift = Vector3d.Exclude(model.surface_v.normalized, lift);
            Vector3d desiredRight = Vector3d.Cross(normalLift, vessel.ReferenceTransform.up).normalized;
            Vector3d actualRight = vessel.ReferenceTransform.right;
            if (desiredRight.sqrMagnitude < 0.5) throw new InvalidOperationException("Invalid roll guidance frame");
            double error = Math.Atan2(Vector3d.Dot(actualRight, normalLift.normalized),
                Vector3d.Dot(desiredRight, actualRight));
            roll.user_controlled = false;
            roll.ApplyControl(state, director.get_desired_roll_v(error));
        }
        private void ApplyVerticalOnly(FlightCtrlState state, double verticalSpeed, double speed, Vector3d up,
            double actualVerticalSpeed)
        {
            ApplyManualRoll(state);
            ApplyVerticalPitch(state, verticalSpeed, speed, up, actualVerticalSpeed);
            ApplyManualYaw(state);
        }
        private void ApplyVerticalPitch(FlightCtrlState state, double verticalSpeed, double speed, Vector3d up,
            double actualVerticalSpeed)
        {
            double projection = Vector3d.Dot(model.pitch_tangent, up);
            if (!NavigationMath.Finite(projection) || projection < 0.5)
                throw new InvalidOperationException("Vertical guidance requires an upright aircraft");
            Vector3d horizontal = Vector3d.Exclude(up, model.surface_v);
            double curvature = horizontal.sqrMagnitude / (vessel.GetWorldPos3D() - vessel.mainBody.position).magnitude;
            double lift = NavigationMath.RequiredVerticalLift(verticalSpeed, actualVerticalSpeed, curvature,
                Vector3d.Dot(model.gravity_acc + model.noninert_acc, up), projection, model.gravity_acc.magnitude);
            double pitchRate = (lift + model.pitch_gravity_acc + model.pitch_noninert_acc) / speed;
            float desiredAoA = director.get_desired_aoa(model.pitch_rot_model_gen, pitchRate, 0);
            if (!NavigationMath.Finite(desiredAoA)) throw new InvalidOperationException("AA pitch model is not ready");
            ControlUtils.neutralize_user_input(state, PITCH);
            pitch.neutral_offset = 0;
            pitch.moderate_aoa = true;
            pitchAoA.user_controlled = false;
            pitchAoA.ApplyControl(state, desiredAoA, 0);
        }
        private void ApplyManualRoll(FlightCtrlState state)
        {
            roll.user_controlled = true;
            roll.ApplyControl(state, 0);
        }
        private void ApplyBankRoll(FlightCtrlState state, double targetBankDegrees, Vector3d up)
        {
            Vector3d forward = Vector3d.Exclude(up, model.surface_v).normalized;
            Vector3d levelRight = Vector3d.Cross(up, forward).normalized;
            if (forward.sqrMagnitude < 0.5 || levelRight.sqrMagnitude < 0.5)
                throw new InvalidOperationException("Bank hold requires horizontal airspeed");
            double radians = NavigationMath.Clamp(targetBankDegrees, -85, 85) * Math.PI / 180.0;
            Vector3d desiredLift = up * Math.Cos(radians) + levelRight * Math.Sin(radians);
            Vector3d desiredRight = Vector3d.Cross(desiredLift, vessel.ReferenceTransform.up).normalized;
            Vector3d actualRight = model.virtualRotation * Vector3.right;
            double error = Math.Atan2(Vector3d.Dot(actualRight, desiredLift),
                Vector3d.Dot(desiredRight, actualRight));
            ControlUtils.neutralize_user_input(state, ROLL);
            roll.user_controlled = false;
            roll.ApplyControl(state, director.get_desired_roll_v(error));
        }
        private void ApplyPitchHold(FlightCtrlState state, double targetPitchDegrees)
        {
            double error = (NavigationMath.Clamp(targetPitchDegrees, -85, 85) - CurrentPitchDegrees()) *
                Math.PI / 180.0;
            float desiredRate = (float)NavigationMath.Clamp(error * 1.5, -0.6, 0.6);
            ControlUtils.neutralize_user_input(state, PITCH);
            pitch.neutral_offset = 0;
            pitch.moderate_aoa = true;
            pitch.user_controlled = false;
            pitch.ApplyControl(state, desiredRate);
        }
        private void ApplyManualPitch(FlightCtrlState state)
        {
            pitch.user_controlled = true;
            pitch.moderate_aoa = savedPitchModeration;
            pitch.neutral_offset = 0;
            if (flyByWire.coord_turn)
            {
                Vector3 levelUp = Vector3.ProjectOnPlane(vessel.ReferenceTransform.position - vessel.mainBody.position,
                    vessel.ReferenceTransform.up).normalized;
                float verticalPitch = Vector3.Dot(levelUp, -vessel.ReferenceTransform.forward);
                if (verticalPitch > 0)
                    pitch.neutral_offset = -Vector3.Dot(model.AngularVel(YAW) * vessel.ReferenceTransform.right,
                        levelUp) / verticalPitch;
            }
            pitch.ApplyControl(state, 0);
        }
        private void ApplyManualYaw(FlightCtrlState state)
        {
            if (flyByWire.rocket_mode)
            {
                yaw.moderate_aoa = savedYawModeration;
                yaw.user_controlled = true;
                yaw.ApplyControl(state, 0);
            }
            else
            {
                yaw.moderate_aoa = savedYawModeration;
                sideslip.user_controlled = true;
                sideslip.ApplyControl(state, 0, 0);
            }
        }
        private void ApplyGuidedYaw(FlightCtrlState state)
        {
            ControlUtils.neutralize_user_input(state, YAW);
            if (flyByWire.rocket_mode)
            {
                yaw.moderate_aoa = true;
                yaw.user_controlled = false;
                yaw.ApplyControl(state, 0);
            }
            else
            {
                yaw.moderate_aoa = true;
                sideslip.user_controlled = false;
                sideslip.ApplyControl(state, 0, 0);
            }
        }
        private void ApplyThrottleGuidance(FlightCtrlState state)
        {
            double target;
            if (selectedThrottleMode == ThrottleGuidanceMode.Acceleration)
            {
                if (!accelerationSetpointInitialized)
                {
                    accelerationSpeedSetpoint = Math.Max(20, model.surface_v_magnitude);
                    accelerationSetpointInitialized = true;
                }
                accelerationSpeedSetpoint = NavigationMath.Clamp(accelerationSpeedSetpoint +
                    accelerationMetersPerSecondSquared * TimeWarp.fixedDeltaTime, 20, 2000);
                target = accelerationSpeedSetpoint;
            }
            else if (selectedThrottleMode == ThrottleGuidanceMode.Landing)
            {
                accelerationSetpointInitialized = false;
                target = LandingSpeedTarget();
            }
            else
            {
                accelerationSetpointInitialized = false;
                target = speedMetersPerSecond;
            }
            commandedSpeedMetersPerSecond = target;
            thrust.ApplyControl(state, (float)target);
        }
        protected override void OnActivate()
        {
            headingDegrees = CurrentTrackDegrees();
            bankDegrees = CurrentBankDegrees();
            pitchDegrees = CurrentPitchDegrees();
            verticalSpeedMetersPerSecond = CurrentVerticalSpeed();
            altitudeMeters = vessel.altitude;
            radarAltitudeMeters = CurrentRadarAltitude();
            headingText = headingDegrees.ToString("F0", CultureInfo.InvariantCulture);
            bankText = bankDegrees.ToString("F1", CultureInfo.InvariantCulture);
            pitchText = pitchDegrees.ToString("F0", CultureInfo.InvariantCulture);
            verticalSpeedText = verticalSpeedMetersPerSecond.ToString("F0", CultureInfo.InvariantCulture);
            altitudeText = altitudeMeters.ToString("F0", CultureInfo.InvariantCulture);
            radarAltitudeText = radarAltitudeMeters.ToString("F0", CultureInfo.InvariantCulture);
            lateralMode = LateralGuidanceMode.Off;
            verticalMode = VerticalGuidanceMode.Off;
            speedEnabled = false;
            accelerationSetpointInitialized = false;
            lateralWasActive = verticalWasActive = false;
            approachWarningShown = false;
            hasLeg = false;
            savedUsePid = thrust.use_pid;
            savedUseBrakes = thrust.use_breaks;
            savedPitchModeration = pitch.moderate_aoa;
            savedYawModeration = yaw.moderate_aoa;
            savedThrustSettings = true;
            thrust.spd_control_enabled = false;
            thrust.use_pid = false;
            thrust.use_breaks = false;
            director.Activate();
            model.Activate();
            thrust.Activate();
            ReloadPlans();
            lastTelemetryTime = 0;
            Debug.Log("[AtmosphereAutopilot NAV/ILS] 0.6.2 controller activated; plans=" + plans.Count);
            MessageManager.post_status_message("AtmosphereAutopilot NAV/ILS enabled in FBW standby");
        }
        protected override void OnDeactivate()
        {
            lateralMode = LateralGuidanceMode.Off;
            verticalMode = VerticalGuidanceMode.Off;
            speedEnabled = false;
            accelerationSetpointInitialized = false;
            thrust.spd_control_enabled = false;
            director.Deactivate();
            pitch.neutral_offset = 0;
            pitch.moderate_aoa = savedPitchModeration;
            yaw.moderate_aoa = savedYawModeration;
            thrust.Deactivate();
            model.Deactivate();
            if (savedThrustSettings)
            {
                thrust.use_pid = savedUsePid;
                thrust.use_breaks = savedUseBrakes;
                savedThrustSettings = false;
            }
            MessageManager.post_status_message("AtmosphereAutopilot NAV/ILS disabled");
        }
        public override void ApplyControl(FlightCtrlState state)
        {
            try { ApplyControlSafe(state); }
            catch (Exception ex)
            {
                Debug.LogError("[AtmosphereAutopilot NAV/ILS] Guidance fault: " + ex);
                DisengageGuidance("AtmosphereAutopilot NAV/ILS fault; FBW restored");
                flyByWire.ApplyControl(state);
            }
        }
        private void ApplyControlSafe(FlightCtrlState state)
        {
            bool lateralActive = lateralMode != LateralGuidanceMode.Off;
            bool verticalActive = verticalMode != VerticalGuidanceMode.Off;
            if (ApproachHandoffRequired())
            {
                DisengageGuidance("Approach handoff height reached; FBW restored");
                flyByWire.ApplyControl(state);
                return;
            }
            if (vessel.LandedOrSplashed || (!lateralActive && !verticalActive))
            {
                lateralWasActive = verticalWasActive = false;
                flyByWire.ApplyControl(state);
                if (speedEnabled && !vessel.LandedOrSplashed)
                    ApplyThrottleGuidance(state);
                status = speedEnabled ? "FBW; Speed control" : "FBW standby";
                return;
            }
            double speed = model.surface_v_magnitude;
            if (!NavigationMath.Finite(speed) || speed < 20)
            {
                DisengageGuidance("Guidance released below 20 m/s");
                flyByWire.ApplyControl(state);
                return;
            }
            GeoPoint position = CurrentPosition();
            hasLeg = false;
            double targetTrack = CurrentTrackDegrees();
            Vector3d up = (vessel.GetWorldPos3D() - vessel.mainBody.position).normalized;
            double actualVerticalSpeed = Vector3d.Dot(model.surface_v, up);
            double targetVerticalSpeed = actualVerticalSpeed;
            if (lateralMode == LateralGuidanceMode.Heading)
                targetTrack = headingDegrees;
            else if (lateralMode == LateralGuidanceMode.Nav)
            {
                EnsureActiveLeg();
                lastLeg = navigator.Update(position, speed, vessel.mainBody.Radius);
                hasLeg = true;
                targetTrack = lastLeg.DesiredTrackDegrees;
            }
            if (verticalMode == VerticalGuidanceMode.Glideslope)
            {
                EnsureActiveLeg();
                if (!hasLeg)
                {
                    lastLeg = navigator.Update(position, speed, vessel.mainBody.Radius);
                    hasLeg = true;
                }
                if (!lastLeg.HasVerticalGuidance)
                    throw new InvalidOperationException("Active Kramax leg has no vertical guidance");
                targetVerticalSpeed = NavigationMath.Clamp(lastLeg.PathSlope * speed +
                    (lastLeg.PathAltitudeMeters - vessel.altitude) / 15.0, -0.2 * speed, 0.2 * speed);
            }
            if (verticalMode == VerticalGuidanceMode.Altitude)
                targetVerticalSpeed = NavigationMath.Clamp((altitudeMeters - vessel.altitude) / 15.0,
                    -0.2 * speed, 0.2 * speed);
            else if (verticalMode == VerticalGuidanceMode.VerticalSpeed)
                targetVerticalSpeed = NavigationMath.Clamp(verticalSpeedMetersPerSecond, -0.2 * speed, 0.2 * speed);
            else if (verticalMode == VerticalGuidanceMode.RadarAltitude)
                targetVerticalSpeed = NavigationMath.Clamp((radarAltitudeMeters - CurrentRadarAltitude()) / 15.0,
                    -0.2 * speed, 0.2 * speed);
            double dt = TimeWarp.fixedDeltaTime;
            if (!lateralWasActive) commandedTrack = CurrentTrackDegrees();
            if (!verticalWasActive) commandedVerticalSpeed = actualVerticalSpeed;
            commandedTrack = NavigationMath.SlewAngle(commandedTrack, targetTrack, 4, dt);
            commandedVerticalSpeed = NavigationMath.MoveTowards(commandedVerticalSpeed, targetVerticalSpeed, 3, dt);
            lateralWasActive = lateralActive;
            verticalWasActive = verticalActive;
            bool bankMode = lateralMode == LateralGuidanceMode.Bank;
            bool pitchMode = verticalMode == VerticalGuidanceMode.Pitch;
            if ((bankMode || pitchMode) && (lateralActive || verticalActive))
                ApplyIndependentModes(state, commandedTrack, commandedVerticalSpeed, speed, up,
                    actualVerticalSpeed, bankMode, pitchMode);
            else if (lateralActive && verticalActive)
                ApplyDirector(state, commandedTrack, commandedVerticalSpeed, speed);
            else if (lateralActive)
                ApplyLateralOnly(state, commandedTrack, speed, up);
            else
                ApplyVerticalOnly(state, commandedVerticalSpeed, speed, up, actualVerticalSpeed);
            if (speedEnabled) ApplyThrottleGuidance(state);
            status = string.Format(CultureInfo.InvariantCulture, "{0} {1:F0} deg; {2} {3:F1} m/s",
                lateralMode, commandedTrack, verticalMode, commandedVerticalSpeed);
            WriteTelemetry(state, speed, actualVerticalSpeed);
        }
        private void ApplyDirector(FlightCtrlState state, double trackDegrees, double verticalSpeed, double speed)
        {
            Vector3d radial = vessel.GetWorldPos3D() - vessel.mainBody.position;
            Vector3d up = radial.normalized;
            Vector3d north = Vector3d.Exclude(up, vessel.mainBody.RotationAxis).normalized;
            Vector3d east = Vector3d.Cross(up, north).normalized;
            if (north.sqrMagnitude < 0.5) throw new InvalidOperationException("Navigation frame unavailable near pole");
            verticalSpeed = NavigationMath.Clamp(verticalSpeed, -0.2 * speed, 0.2 * speed);
            double radians = trackDegrees * Math.PI / 180.0;
            Vector3d horizontal = north * Math.Cos(radians) + east * Math.Sin(radians);
            double horizontalSpeed = Math.Sqrt(Math.Max(1, speed * speed - verticalSpeed * verticalSpeed));
            Vector3d desiredVelocity = horizontal * horizontalSpeed + up * verticalSpeed;
            Vector3d actualHorizontal = Vector3d.Exclude(up, model.surface_v);
            Vector3d levelAcceleration = -up * actualHorizontal.sqrMagnitude / radial.magnitude;
            ControlUtils.neutralize_user_input(state, PITCH);
            ControlUtils.neutralize_user_input(state, ROLL);
            ControlUtils.neutralize_user_input(state, YAW);
            double oldStrength = director.strength;
            bool oldInverted = director.allow_spine_down;
            try
            {
                director.strength = Math.Min(oldStrength, 0.35);
                director.allow_spine_down = false;
                director.ApplyControl(state, desiredVelocity, levelAcceleration);
            }
            finally
            {
                director.strength = oldStrength;
                director.allow_spine_down = oldInverted;
            }
        }
        private void ReloadPlans()
        {
            loadErrors.Clear();
            var loaded = new List<NavPlan>();
            foreach (NavPlan plan in KramaxPlanLoader.LoadInstallation(KSPUtil.ApplicationRootPath, loadErrors))
                if (string.Equals(plan.Body, vessel.mainBody.bodyName, StringComparison.OrdinalIgnoreCase))
                    loaded.Add(plan);
            foreach (NavPlan plan in NativePlanLoader.LoadDirectory(KSPUtil.ApplicationRootPath +
                "GameData/AtmosphereAutopilotNav/FlightPlans", loadErrors))
                if (string.Equals(plan.Body, vessel.mainBody.bodyName, StringComparison.OrdinalIgnoreCase) &&
                    !loaded.Exists(existing => string.Equals(existing.Name, plan.Name, StringComparison.OrdinalIgnoreCase)))
                    loaded.Add(plan);
            plans = loaded;
            if (planIndex >= plans.Count) planIndex = Math.Max(0, plans.Count - 1);
            SelectCurrentPlan();
        }
        private void SelectCurrentPlan()
        {
            hasLeg = false;
            if (plans.Count == 0) navigator.Clear();
            else
            {
                navigator.SetPlan(plans[planIndex]);
                planNameText = navigator.Plan.Name;
                planDescriptionText = navigator.Plan.Description;
                loadedPlanName = navigator.Plan.Name;
            }
        }

        private void SaveCurrentPlan()
        {
            if (navigator.Plan == null) return;
            string name = (planNameText ?? "").Trim();
            if (name.Length == 0)
            {
                ScreenMessages.PostScreenMessage("Flight plan name is required", 5f, ScreenMessageStyle.UPPER_CENTER);
                return;
            }
            string description = string.IsNullOrWhiteSpace(planDescriptionText) ? name : planDescriptionText.Trim();
            NavPlan edited = new NavPlan("kramax:" + navigator.Plan.Body + ":" + name, name,
                navigator.Plan.Body, navigator.Plan.Waypoints, navigator.Plan.Approaches, description);
            try
            {
                KramaxPlanLoader.SaveUserPlan(KSPUtil.ApplicationRootPath, edited, loadedPlanName);
                string selectedName = name;
                ReloadPlans();
                int selected = -1;
                for (int i = 0; i < plans.Count; i++)
                    if (string.Equals(plans[i].Name, selectedName, StringComparison.OrdinalIgnoreCase)) { selected = i; break; }
                if (selected >= 0) { planIndex = selected; SelectCurrentPlan(); }
                ScreenMessages.PostScreenMessage("Flight plan updated", 5f, ScreenMessageStyle.UPPER_CENTER);
            }
            catch (Exception ex)
            {
                loadErrors.Add("Save: " + ex.Message);
                ScreenMessages.PostScreenMessage("Flight plan save failed", 5f, ScreenMessageStyle.UPPER_CENTER);
            }
        }
        private void EnsureSelectedPlan()
        {
            if (plans.Count == 0 || navigator.Plan == null)
                throw new InvalidOperationException("No native flight plan loaded");
            if (!string.Equals(navigator.Plan.Body, vessel.mainBody.bodyName, StringComparison.OrdinalIgnoreCase))
                throw new InvalidOperationException("Selected plan is for " + navigator.Plan.Body);
        }
        private void EnsureActiveLeg()
        {
            EnsureSelectedPlan();
            if (navigator.Complete) throw new InvalidOperationException("Selected flight plan is complete");
        }
        private GeoPoint CurrentPosition() => new GeoPoint(vessel.latitude, vessel.longitude, vessel.altitude);
        private double CurrentTrackDegrees()
        {
            Vector3d up = (vessel.GetWorldPos3D() - vessel.mainBody.position).normalized;
            Vector3d north = Vector3d.Exclude(up, vessel.mainBody.RotationAxis).normalized;
            Vector3d east = Vector3d.Cross(up, north).normalized;
            Vector3d horizontalVelocity = Vector3d.Exclude(up, vessel.srf_velocity);
            if (north.sqrMagnitude < 0.5 || horizontalVelocity.sqrMagnitude < 1)
                return NavigationMath.Finite(commandedTrack) ? commandedTrack : 0;
            return NavigationMath.Wrap360(Math.Atan2(Vector3d.Dot(horizontalVelocity, east),
                Vector3d.Dot(horizontalVelocity, north)) * 180.0 / Math.PI);
        }
        private double CurrentVerticalSpeed()
        {
            Vector3d up = (vessel.GetWorldPos3D() - vessel.mainBody.position).normalized;
            return Vector3d.Dot(model.surface_v, up);
        }
        private double CurrentRadarAltitude()
        {
            return vessel.altitude - Math.Max(0, vessel.terrainAltitude);
        }
        private double CurrentPitchDegrees()
        {
            Vector3d up = (vessel.GetWorldPos3D() - vessel.mainBody.position).normalized;
            double sine = NavigationMath.Clamp(Vector3d.Dot(vessel.ReferenceTransform.up, up), -1, 1);
            return Math.Asin(sine) * 180.0 / Math.PI;
        }
        private double CurrentBankDegrees()
        {
            Vector3d up = (vessel.GetWorldPos3D() - vessel.mainBody.position).normalized;
            Vector3d forward = Vector3d.Exclude(up, vessel.ReferenceTransform.up).normalized;
            Vector3d levelRight = Vector3d.Cross(up, forward).normalized;
            Vector3d actualRight = model.virtualRotation * Vector3.right;
            if (levelRight.sqrMagnitude < 0.5) return 0;
            return Math.Atan2(-Vector3d.Dot(actualRight, up), Vector3d.Dot(actualRight, levelRight)) *
                180.0 / Math.PI;
        }
        private double CurrentLongitudinalAcceleration()
        {
            if (model.surface_v.sqrMagnitude < 1) return 0;
            return Vector3d.Dot(model.sum_acc, model.surface_v.normalized);
        }
        private double LandingSpeedTarget()
        {
            if (navigator.Plan == null || navigator.Complete) return speedMetersPerSecond;
            for (int i = navigator.TargetIndex; i < navigator.Plan.Waypoints.Count; i++)
            {
                WaypointRole role = navigator.Plan.Waypoints[i].Role;
                if (role == WaypointRole.InitialApproachFix) return iafSpeedMetersPerSecond;
                if (role == WaypointRole.FinalApproachFix) return fafSpeedMetersPerSecond;
                if (role == WaypointRole.Runway || role == WaypointRole.Stop) return finalSpeedMetersPerSecond;
            }
            return speedMetersPerSecond;
        }
        private bool ApproachHandoffRequired()
        {
            if (navigator.Plan == null || navigator.Complete) return false;
            bool approachGuidance = lateralMode == LateralGuidanceMode.Nav ||
                verticalMode == VerticalGuidanceMode.Glideslope ||
                (speedEnabled && selectedThrottleMode == ThrottleGuidanceMode.Landing);
            if (!approachGuidance || navigator.Plan.Waypoints[navigator.TargetIndex].Role != WaypointRole.Runway)
                return false;
            double radarAltitude = CurrentRadarAltitude();
            if (!approachWarningShown && radarAltitude <= 300)
            {
                approachWarningShown = true;
                ScreenMessages.PostScreenMessage("AtmosphereAutopilot NAV/ILS: prepare for manual landing handoff.", 8f,
                    ScreenMessageStyle.UPPER_CENTER);
            }
            return radarAltitude <= 150;
        }
        private void DisengageGuidance(string message)
        {
            lateralMode = LateralGuidanceMode.Off;
            verticalMode = VerticalGuidanceMode.Off;
            speedEnabled = false;
            lateralWasActive = verticalWasActive = false;
            accelerationSetpointInitialized = false;
            thrust.spd_control_enabled = false;
            status = message;
            MessageManager.post_status_message(message);
        }
        private void WriteTelemetry(FlightCtrlState state, double speed, double actualVerticalSpeed)
        {
            double now = Planetarium.GetUniversalTime();
            if (now - lastTelemetryTime < 2) return;
            lastTelemetryTime = now;
            string plan = navigator.Plan == null ? "none" : navigator.Plan.Id;
            Debug.Log(string.Format(CultureInfo.InvariantCulture,
                "[AtmosphereAutopilot NAV/ILS] t={0:F1} plan={1} leg={2} modes={3}/{4}/{5} trackCmd={6:F2} track={7:F2} vsCmd={8:F2} vs={9:F2} speedCmd={10:F1} speed={11:F1} xtk={12:F1} dthr={13:F1} roll={14:F3} pitch={15:F3} yaw={16:F3} throttle={17:F3}",
                now, plan, navigator.TargetIndex, lateralMode, verticalMode, speedEnabled,
                commandedTrack, CurrentTrackDegrees(), commandedVerticalSpeed, actualVerticalSpeed,
                commandedSpeedMetersPerSecond, speed, hasLeg ? lastLeg.CrossTrackMeters : 0,
                navigator.Plan != null && !navigator.Complete &&
                    navigator.Plan.Waypoints[navigator.TargetIndex].Role == WaypointRole.Runway && hasLeg
                    ? lastLeg.DistanceToTargetMeters : -1,
                state.roll, state.pitch, state.yaw, state.mainThrottle));
        }
        protected override void _drawGUI(int id)
        {
            EnsureStyles();
            close_button();
            GUILayout.BeginVertical(GUILayout.Width(380));
            GUILayout.Label("Status: " + status, greenLabelStyle);
            DrawLateralControls();
            DrawVerticalControls();
            DrawSpeedControls();
            DrawPlanControls();
            DrawApproachSpeeds();
            GUILayout.EndVertical();
            GUI.DragWindow();
        }
        private void EnsureStyles()
        {
            if (sectionStyle != null) return;
            sectionStyle = new GUIStyle(GUI.skin.button);
            sectionStyle.normal.textColor = Color.white;
            sectionStyle.hover.textColor = Color.white;
            activeSectionStyle = new GUIStyle(GUI.skin.button);
            activeSectionStyle.normal.textColor = Color.white;
            activeSectionStyle.hover.textColor = Color.white;
            modeStyle = new GUIStyle(GUI.skin.button);
            selectedModeStyle = new GUIStyle(GUI.skin.button);
            selectedModeStyle.normal.textColor = Color.green;
            selectedModeStyle.hover.textColor = new Color(0.4f, 1f, 0.4f);
            valueStyle = new GUIStyle(GUI.skin.textField);
            valueStyle.normal.textColor = Color.white;
            valueStyle.focused.textColor = Color.white;
            valueStyle.hover.textColor = valueStyle.active.textColor = Color.white;
            valueStyle.alignment = TextAnchor.MiddleLeft;
            greenLabelStyle = new GUIStyle(GUI.skin.label);
            greenLabelStyle.normal.textColor = Color.green;
            sectionStyle.active.textColor = sectionStyle.focused.textColor = Color.white;
            activeSectionStyle.active.textColor = activeSectionStyle.focused.textColor = Color.white;
            compactValueStyle = new GUIStyle(valueStyle) { wordWrap = false, fontSize = 12,
                clipping = TextClipping.Clip, alignment = TextAnchor.MiddleCenter,
                margin = new RectOffset(2, 2, 3, 3), padding = new RectOffset(3, 3, 1, 1) };
            compactValueStyle.normal.textColor = compactValueStyle.focused.textColor =
                compactValueStyle.hover.textColor = compactValueStyle.active.textColor = Color.green;
            compactLabelStyle = new GUIStyle(GUI.skin.label) { wordWrap = false, fontSize = 12, alignment = TextAnchor.MiddleLeft,
                margin = new RectOffset(2, 2, 3, 3), padding = new RectOffset(0, 0, 1, 1) };
            compactButtonStyle = new GUIStyle(GUI.skin.button) { wordWrap = false, fontSize = 12, alignment = TextAnchor.MiddleCenter,
                margin = new RectOffset(2, 2, 3, 3), padding = new RectOffset(1, 1, 0, 0) };
            compactButtonStyle.normal.textColor = Color.green;
            incrementStyle = new GUIStyle(GUI.skin.button) { fontSize = 10,
                margin = new RectOffset(0, 2, 0, 0), padding = new RectOffset(0, 0, 0, 0) };
        }
        private bool SectionButton(string label, bool active)
        {
            Color old = GUI.backgroundColor;
            GUI.backgroundColor = active ? new Color(0.05f, 0.55f, 0.6f) : new Color(0.65f, 0.22f, 0.16f);
            bool pressed = GUILayout.Button(active ? label + " (Active)" : label, active ? activeSectionStyle : sectionStyle, GUILayout.Height(22));
            GUI.backgroundColor = old;
            return pressed;
        }
        private bool ModeButton(string label, bool selected, params GUILayoutOption[] options)
        {
            Color old = GUI.backgroundColor;
            if (selected) GUI.backgroundColor = new Color(0.12f, 0.55f, 0.58f);
            bool pressed = GUILayout.Button(label, selected ? selectedModeStyle : modeStyle, options);
            GUI.backgroundColor = old;
            return pressed;
        }
        // Drag upward to hide the mode row, downward to reveal it. Targets stay accessible.
        private void Separator(ref bool modesVisible, int hint)
        {
            GUILayout.Space(5);
            Rect rect = GUILayoutUtility.GetRect(0, 8, GUILayout.ExpandWidth(true));
            int control = GUIUtility.GetControlID(hint, FocusType.Passive, rect);
            Event e = Event.current;
            if (e.type == EventType.Repaint) GUI.skin.button.Draw(rect, GUIContent.none, false, false, false, false);
            if (e.type == EventType.MouseDown && e.button == 0 && rect.Contains(e.mousePosition))
            {
                GUIUtility.hotControl = control;
                separatorDragDistance = 0;
                e.Use();
            }
            if (GUIUtility.hotControl != control) return;
            if (e.type == EventType.MouseDrag)
            {
                separatorDragDistance += e.delta.y;
                if (separatorDragDistance < -12) modesVisible = false;
                if (separatorDragDistance > 12) modesVisible = true;
                e.Use();
            }
            if (e.type == EventType.MouseUp)
            {
                GUIUtility.hotControl = 0;
                e.Use();
            }
        }

        private void DrawPlanControls()
        {
            GUILayout.BeginHorizontal();
            Color old = GUI.backgroundColor;
            GUI.backgroundColor = new Color(0.05f, 0.55f, 0.6f);
            if (GUILayout.Button("Flight Plan", sectionStyle)) flightPlanExpanded = !flightPlanExpanded;
            GUI.backgroundColor = new Color(0.55f, 0.28f, 0.16f);
            if (GUILayout.Button("Load/Store", sectionStyle)) planManagerShown = !planManagerShown;
            GUI.backgroundColor = old;
            GUILayout.EndHorizontal();
            if (!flightPlanExpanded) return;
            GUILayout.BeginHorizontal();
            GUILayout.Label("Plan:", compactLabelStyle, GUILayout.Width(80));
            GUILayout.Label(navigator.Plan == null ? "<none>" : navigator.Plan.Name, compactValueStyle, GUILayout.Height(20));
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            GUILayout.Label("Appr Mode:", compactLabelStyle, GUILayout.Width(80));
            GUILayout.Label(CurrentApproachMode(), compactValueStyle, GUILayout.Width(65), GUILayout.Height(20));
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            GUILayout.Label("Position:", compactLabelStyle, GUILayout.Width(80));
            GUILayout.Label(Math.Abs(vessel.latitude).ToString("F3", CultureInfo.InvariantCulture) + "°" +
                (vessel.latitude < 0 ? "S" : "N"), compactValueStyle, GUILayout.Width(65), GUILayout.Height(20));
            GUILayout.Label(Math.Abs(vessel.longitude).ToString("F3", CultureInfo.InvariantCulture) + "°" +
                (vessel.longitude < 0 ? "W" : "E"), compactValueStyle, GUILayout.Width(65), GUILayout.Height(20));
            GUILayout.Label(string.Format(CultureInfo.InvariantCulture, "{0:F0}m ({1:F0}m)", vessel.altitude,
                CurrentRadarAltitude()), compactValueStyle, GUILayout.ExpandWidth(true), GUILayout.Height(20));
            GUILayout.EndHorizontal();
            if (loadErrors.Count > 0) GUILayout.Label("Load error: " + loadErrors[0]);
            if (navigator.Plan == null) GUILayout.Label("No Flight Plan");
            else
            {
                GUILayout.BeginHorizontal();
                string[] labels = { "Leg", "Loc", "Alt(m)", "Hdg", "km", "" };
                for (int i = 0; i < labels.Length; i++)
                    GUILayout.Label(labels[i], compactLabelStyle, GUILayout.Width(PlanColumnWidths[i]), GUILayout.Height(20));
                GUILayout.EndHorizontal();
                for (int i = 0; i < navigator.Plan.Waypoints.Count; i++) DrawWaypointRow(i);
            }
        }

        private void DrawLateralControls()
        {
            if (SectionButton("Roll", lateralMode != LateralGuidanceMode.Off)) ToggleRoll();
            if (rollModesVisible)
            {
                GUILayout.BeginHorizontal();
                if (ModeButton("Bank", selectedLateralMode == LateralGuidanceMode.Bank, GUILayout.ExpandWidth(true)))
                    SelectLateral(LateralGuidanceMode.Bank);
                if (ModeButton("Hdg", selectedLateralMode == LateralGuidanceMode.Heading, GUILayout.ExpandWidth(true)))
                    SelectLateral(LateralGuidanceMode.Heading);
                if (ModeButton("NAV", selectedLateralMode == LateralGuidanceMode.Nav, GUILayout.ExpandWidth(true)))
                    SelectLateral(LateralGuidanceMode.Nav);
                GUILayout.EndHorizontal();
            }
            if (selectedLateralMode == LateralGuidanceMode.Bank)
                DrawTargetRow("Target Bank:", ref bankText, 1, v => bankDegrees = NavigationMath.Clamp(v, -85, 85), CurrentBankDegrees());
            else
                DrawTargetRow("Target Hdg:", ref headingText, 5, v => headingDegrees = NavigationMath.Wrap360(v), CurrentTrackDegrees());
            Separator(ref rollModesVisible, 701);
        }

        private void DrawVerticalControls()
        {
            if (SectionButton("Vertical", verticalMode != VerticalGuidanceMode.Off)) ToggleVertical();
            if (verticalModesVisible)
            {
                GUILayout.BeginHorizontal();
                if (ModeButton("Pitch", selectedVerticalMode == VerticalGuidanceMode.Pitch, GUILayout.ExpandWidth(true)))
                    SelectVertical(VerticalGuidanceMode.Pitch);
                if (ModeButton("VSpd", selectedVerticalMode == VerticalGuidanceMode.VerticalSpeed, GUILayout.ExpandWidth(true)))
                    SelectVertical(VerticalGuidanceMode.VerticalSpeed);
                if (ModeButton("Alt", selectedVerticalMode == VerticalGuidanceMode.Altitude, GUILayout.ExpandWidth(true)))
                    SelectVertical(VerticalGuidanceMode.Altitude);
                if (ModeButton("RAlt", selectedVerticalMode == VerticalGuidanceMode.RadarAltitude, GUILayout.ExpandWidth(true)))
                    SelectVertical(VerticalGuidanceMode.RadarAltitude);
                if (ModeButton("GS", selectedVerticalMode == VerticalGuidanceMode.Glideslope, GUILayout.ExpandWidth(true)))
                    SelectVertical(VerticalGuidanceMode.Glideslope);
                GUILayout.EndHorizontal();
            }
            DrawVerticalTargetRow();
            Separator(ref verticalModesVisible, 702);
        }

        private void DrawSpeedControls()
        {
            if (SectionButton("Auto-Throttle", speedEnabled)) ToggleThrottle();
            if (throttleModesVisible)
            {
                GUILayout.BeginHorizontal();
                if (ModeButton("Vel", selectedThrottleMode == ThrottleGuidanceMode.Velocity, GUILayout.ExpandWidth(true)))
                    SelectThrottle(ThrottleGuidanceMode.Velocity);
                if (ModeButton("Acc", selectedThrottleMode == ThrottleGuidanceMode.Acceleration, GUILayout.ExpandWidth(true)))
                    SelectThrottle(ThrottleGuidanceMode.Acceleration);
                if (ModeButton("Land", selectedThrottleMode == ThrottleGuidanceMode.Landing, GUILayout.ExpandWidth(true)))
                    SelectThrottle(ThrottleGuidanceMode.Landing);
                GUILayout.EndHorizontal();
            }
            if (selectedThrottleMode == ThrottleGuidanceMode.Acceleration)
                DrawTargetRow("Target Acc:", ref accelerationText, 10, v => accelerationMetersPerSecondSquared = v, CurrentLongitudinalAcceleration());
            else if (selectedThrottleMode == ThrottleGuidanceMode.Landing)
                DrawLandingSpeedRow();
            else
                DrawTargetRow("Target Speed:", ref speedText, 10, v => speedMetersPerSecond = Math.Max(20, v), model.surface_v_magnitude);
            Separator(ref throttleModesVisible, 703);
        }

        private void ToggleRoll()
        {
            if (lateralMode != LateralGuidanceMode.Off) lateralMode = LateralGuidanceMode.Off;
            else lateralMode = selectedLateralMode;
            lateralWasActive = false;
        }
        private void SelectLateral(LateralGuidanceMode mode)
        {
            selectedLateralMode = mode;
            if (lateralMode != LateralGuidanceMode.Off) lateralMode = mode;
            lateralWasActive = false;
        }
        private void ToggleVertical()
        {
            if (verticalMode != VerticalGuidanceMode.Off) verticalMode = VerticalGuidanceMode.Off;
            else verticalMode = selectedVerticalMode;
            verticalWasActive = false;
        }
        private void SelectVertical(VerticalGuidanceMode mode)
        {
            selectedVerticalMode = mode;
            if (verticalMode != VerticalGuidanceMode.Off) verticalMode = mode;
            verticalWasActive = false;
        }
        private void ToggleThrottle()
        {
            if (speedEnabled) speedEnabled = false;
            else speedEnabled = true;
            accelerationSetpointInitialized = false;
        }
        private void SelectThrottle(ThrottleGuidanceMode mode)
        {
            selectedThrottleMode = mode;
            accelerationSetpointInitialized = false;
        }
        private void DrawTargetRow(string label, ref string text, double step,
            Func<double, double> apply, double? actual = null)
        {
            GUILayout.BeginHorizontal();
            bool commit = GUILayout.Button(label, GUILayout.Width(118), GUILayout.Height(24));
            text = GUILayout.TextField(text, valueStyle, GUILayout.Width(88), GUILayout.Height(24));
            GUILayout.BeginVertical(GUILayout.Width(20));
            bool increase = GUILayout.Button("+", incrementStyle, GUILayout.Width(20), GUILayout.Height(12));
            bool decrease = GUILayout.Button("-", incrementStyle, GUILayout.Width(20), GUILayout.Height(12));
            GUILayout.EndVertical();
            double value;
            if (double.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out value) && NavigationMath.Finite(value))
            {
                if (increase || decrease)
                    text = (value + (increase ? step : -step)).ToString("F1", CultureInfo.InvariantCulture);
                if (commit)
                {
                    text = apply(value).ToString("F1", CultureInfo.InvariantCulture);
                    string confirmation;
                    switch (label)
                    {
                        case "Target Hdg:": confirmation = "Heading updated to " + headingDegrees.ToString("F0", CultureInfo.InvariantCulture) + "°"; break;
                        case "Target Alt:": confirmation = "Target Altitude updated"; break;
                        case "Target RAlt:": confirmation = "Target Radar Alt updated"; break;
                        case "Target VSpd:": confirmation = "Target Speed updated"; break;
                        case "Target Acc:": confirmation = "Target Accel updated"; break;
                        case "Set IAF Speed:": confirmation = "To IAF speed updated"; break;
                        case "Set FAF Speed:": confirmation = "To FAF speed updated"; break;
                        case "Set Final Speed:": confirmation = "Final approach speed updated"; break;
                        default: confirmation = label.TrimEnd(':') + " updated"; break;
                    }
                    ScreenMessages.PostScreenMessage(confirmation);
                    GUI.FocusControl(null);
                }
            }
            GUILayout.FlexibleSpace();
            if (actual.HasValue)
                GUILayout.Label(actual.Value.ToString("F1", CultureInfo.InvariantCulture), compactValueStyle,
                    GUILayout.Width(60), GUILayout.Height(22));
            GUILayout.EndHorizontal();
        }

        private void DrawLandingSpeedRow()
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label("Auto-Land Speed:", GUILayout.Width(118));
            GUILayout.Label(LandingSpeedTarget().ToString("F1", CultureInfo.InvariantCulture), compactValueStyle,
                GUILayout.Width(92));
            GUILayout.Label(model.surface_v_magnitude.ToString("F1", CultureInfo.InvariantCulture),
                compactValueStyle, GUILayout.Width(70));
            GUILayout.EndHorizontal();
        }
        private void DrawVerticalTargetRow()
        {
            switch (selectedVerticalMode)
            {
                case VerticalGuidanceMode.Pitch:
                    DrawTargetRow("Target Pitch:", ref pitchText, 1, v => pitchDegrees = NavigationMath.Clamp(v, -85, 85), CurrentPitchDegrees()); break;
                case VerticalGuidanceMode.VerticalSpeed:
                    DrawTargetRow("Target VSpd:", ref verticalSpeedText, 10, v => verticalSpeedMetersPerSecond = v, CurrentVerticalSpeed()); break;
                case VerticalGuidanceMode.RadarAltitude:
                    DrawTargetRow("Target RAlt:", ref radarAltitudeText, 100, v => radarAltitudeMeters = Math.Max(0, v), CurrentRadarAltitude()); break;
                case VerticalGuidanceMode.Glideslope:
                    GUILayout.BeginHorizontal();
                    GUILayout.Label("Active leg glideslope", greenLabelStyle);
                    GUILayout.EndHorizontal();
                    break;
                default:
                    DrawTargetRow("Target Alt:", ref altitudeText, 500, v => altitudeMeters = v, vessel.altitude); break;
            }
        }
        private void DrawNavigationStatus()
        {
            if (hasLeg)
                GUILayout.Label(string.Format(CultureInfo.InvariantCulture,
                    "NAV {0}->{1}  XTK {2:F0}m  DTG {3:F0}m", lastLeg.FromId, lastLeg.ToId,
                    lastLeg.CrossTrackMeters, lastLeg.DistanceToTargetMeters));
            if (hasLeg && verticalMode == VerticalGuidanceMode.Glideslope)
                GUILayout.Label(string.Format(CultureInfo.InvariantCulture,
                    "GS {0}->{1}  PATH {2:F0}m  ERR {3:F0}m", lastLeg.FromId, lastLeg.ToId,
                    lastLeg.PathAltitudeMeters, vessel.altitude - lastLeg.PathAltitudeMeters));
        }
        private void DrawApproachSpeeds()
        {
            GUILayout.Space(5);
            GUILayout.Box(GUIContent.none, GUI.skin.button, GUILayout.Height(8), GUILayout.ExpandWidth(true));
            Color old = GUI.backgroundColor;
            GUI.backgroundColor = new Color(0.05f, 0.55f, 0.6f);
            if (GUILayout.Button("Approach Speeds", sectionStyle)) approachSpeedsExpanded = !approachSpeedsExpanded;
            GUI.backgroundColor = old;
            if (!approachSpeedsExpanded) return;
            DrawTargetRow("Set IAF Speed:", ref iafSpeedText, 5, v => iafSpeedMetersPerSecond = Math.Max(20, v));
            GUILayout.Space(4);
            DrawTargetRow("Set FAF Speed:", ref fafSpeedText, 5, v => fafSpeedMetersPerSecond = Math.Max(20, v));
            GUILayout.Space(4);
            DrawTargetRow("Set Final Speed:", ref finalSpeedText, 5, v => finalSpeedMetersPerSecond = Math.Max(20, v));
        }

        private static readonly float[] PlanColumnWidths = { 38, 122, 48, 32, 54, 36 };

        private void DrawWaypointRow(int index)
        {
            CoreWaypoint waypoint = navigator.Plan.Waypoints[index];
            bool active = index == navigator.TargetIndex;
            GUILayout.BeginHorizontal();
            if (GUILayout.Button(active ? "Active" : "D➔", compactButtonStyle,
                GUILayout.Width(PlanColumnWidths[0]), GUILayout.Height(20))) navigator.ActivateLeg(index);
            GUILayout.Label(new GUIContent(waypoint.Id, waypoint.Id), compactValueStyle,
                GUILayout.Width(PlanColumnWidths[1]), GUILayout.Height(20));
            GUILayout.Label(waypoint.Position.AltitudeMeters.ToString("F0", CultureInfo.InvariantCulture), compactValueStyle,
                GUILayout.Width(PlanColumnWidths[2]), GUILayout.Height(20));
            double heading = index == 0 ? NavigationMath.InitialBearingDegrees(CurrentPosition(), waypoint.Position) :
                NavigationMath.InitialBearingDegrees(navigator.Plan.Waypoints[index - 1].Position, waypoint.Position);
            GUILayout.Label(heading.ToString("F0", CultureInfo.InvariantCulture) + "°", compactValueStyle,
                GUILayout.Width(PlanColumnWidths[3]), GUILayout.Height(20));
            double distance = NavigationMath.DistanceMeters(CurrentPosition(), waypoint.Position, vessel.mainBody.Radius) / 1000.0;
            GUILayout.Label(distance.ToString("F1", CultureInfo.InvariantCulture), compactValueStyle,
                GUILayout.Width(PlanColumnWidths[4]), GUILayout.Height(20));
            string role = RoleText(waypoint.Role);
            GUILayout.Label(role, role.Length == 0 ? compactLabelStyle : compactValueStyle,
                GUILayout.Width(PlanColumnWidths[5]), GUILayout.Height(20));
            GUILayout.EndHorizontal();
        }

        private string CurrentApproachMode()
        {
            if (vessel.LandedOrSplashed) return "Ground";
            if (navigator.Plan == null || navigator.Complete) return "Cruise";
            switch (navigator.Plan.Waypoints[navigator.TargetIndex].Role)
            {
                case WaypointRole.InitialApproachFix: return "IAF";
                case WaypointRole.FinalApproachFix: return "FAF";
                case WaypointRole.Runway: return "Final";
                case WaypointRole.Stop: return "Runway";
                default: return "Cruise";
            }
        }
        private static string RoleText(WaypointRole role)
        {
            switch (role)
            {
                case WaypointRole.InitialApproachFix: return "IAF";
                case WaypointRole.FinalApproachFix: return "FAF";
                case WaypointRole.Runway: return "Final";
                case WaypointRole.Stop: return "RW";
                default: return "";
            }
        }
        protected override void OnGUICustomAlways()
        {
            // Recalculate size only when the layout changes; preserve the window rect for dragging and closing.
            int signature = (rollModesVisible ? 1 : 0) | (verticalModesVisible ? 2 : 0) |
                (throttleModesVisible ? 4 : 0) | (flightPlanExpanded ? 8 : 0) |
                (approachSpeedsExpanded ? 16 : 0) |
                ((navigator.Plan == null ? 0 : navigator.Plan.Waypoints.Count + 1) << 5);
            if (signature != layoutSignature)
            {
                window.width = 0;
                window.height = 0;
                layoutSignature = signature;
            }
            if (IsShown() && !AtmosphereAutopilot.AtmosphereAutopilot.UIHidden && vessel == FlightGlobals.ActiveVessel)
            {
                bool showHorizontal = selectedLateralMode == LateralGuidanceMode.Nav;
                bool showVertical = selectedVerticalMode == VerticalGuidanceMode.Glideslope;
                double horizontal = double.NaN, vertical = double.NaN;
                if ((showHorizontal || showVertical) && !navigator.Complete &&
                    string.Equals(navigator.Plan.Body, vessel.mainBody.bodyName, StringComparison.OrdinalIgnoreCase))
                {
                    LegGuidance preview = navigator.Preview(CurrentPosition(), vessel.srfSpeed, vessel.mainBody.Radius);
                    horizontal = preview.CrossTrackMeters;
                    if (preview.HasVerticalGuidance) vertical = vessel.altitude - preview.PathAltitudeMeters;
                }
                deviationOverlay.Draw(showHorizontal, showVertical, horizontal, vertical);
            }
            else deviationOverlay.CancelDrag();
            if (planManagerShown && !AtmosphereAutopilot.AtmosphereAutopilot.UIHidden)
                planManagerRect = GUILayout.Window(88439962, planManagerRect, DrawPlanManager, "Flight Plan Management");
        }
        private void DrawPlanManager(int id)
        {
            if (GUI.Button(new Rect(planManagerRect.width - 18, 1, 16, 16), "x", GUIStyles.toggleButtonStyle))
                planManagerShown = false;
            GUILayout.BeginHorizontal();
            GUILayout.Label("Flight Plan Management");
            if (GUILayout.Button("Refresh", GUILayout.Width(100))) ReloadPlans();
            GUILayout.EndHorizontal();
            GUILayout.Label("Planet: " + vessel.mainBody.bodyName);
            if (navigator.Plan == null) GUILayout.Label("<no currently loaded plan>");
            else
            {
                GUILayout.BeginHorizontal();
                GUILayout.Label("Current plan name:", GUILayout.Width(120));
                planNameText = GUILayout.TextField(planNameText, GUILayout.Width(175));
                GUILayout.EndHorizontal();
                GUILayout.BeginHorizontal();
                GUILayout.Label("Current plan info:", GUILayout.Width(120));
                planDescriptionText = GUILayout.TextField(planDescriptionText, GUILayout.Width(175));
                GUILayout.EndHorizontal();
                GUILayout.BeginHorizontal();
                GUILayout.Label("Modify Plan:", GUILayout.Width(120));
                if (GUILayout.Button("Update/Rename", GUILayout.Width(175))) SaveCurrentPlan();
                GUILayout.EndHorizontal();
            }
            GUILayout.Space(6);
            GUILayout.Label("Available Plans");
            planScroll = GUILayout.BeginScrollView(planScroll, GUILayout.Height(300));
            for (int i = 0; i < plans.Count; i++)
            {
                if (GUILayout.Button(plans[i].Name))
                {
                    planIndex = i;
                    SelectCurrentPlan();
                }
            }
            GUILayout.EndScrollView();
            GUI.DragWindow();
        }
    }
}
