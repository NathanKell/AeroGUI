/*by NathanKell
 * License: MIT
 */
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Reflection;
using UnityEngine;
using KSP;

namespace KSPExp
{
    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public class AeroGUI : MonoBehaviour
    {
        const double DEG2RAD = Math.PI / 180.0;
        const double RAD2DEG = 180.0 / Math.PI;
        // GUI bits
        public static Rect windowPos = new Rect(100, 100, 0, 0);
        public static Rect windowPosThermal = new Rect(200, 200, 0, 0);
        public static bool guiEnabled = false;
        public KeyCode key = KeyCode.I;
        public bool useModifier = true;
        public bool winterOwlModeOff = true;

        public static bool showThermal = false;

        private const double bodyEmissiveScalarS0Front = 0.782048841d;
        private const double bodyEmissiveScalarS0Back = 0.093081228d;
        private const double bodyEmissiveScalarS1 = 0.87513007d;
        private const double bodyEmissiveScalarS0Top = 0.398806364d;
        private const double bodyEmissiveScalarS1Top = 0.797612728d;

        // Thermal bits
        double scaleHeightMult, solarFlux, backgroundRadTemp, bodyAlbedoFlux, bodyEmissiveFlux, bodySunFlux, effectiveFaceTemp, bodyTemperature, sunDot, atmosphereTemperatureOffset, altTempMult, latitude, latTempMod, axialTempMod, solarAMMult, finalAtmoMod, sunFinalMult, diurnalRange;
        bool inSun;

        // display forces
        public double terminalV, alpha, sideslip, soundSpeed, mach, eas, thrust, climbrate, lift, drag, lidForce, dragUpForce, pLift, pDrag, liftUp, grav, ldRatio, Q, pressure, density, ambientTemp, shockTemp;

        // thermal counter
        public double convectiveTotal = 0d;

        // via regex
        

        public void Start()
        {
            enabled = true;
            scaleHeightMult = -Math.Log(0.000001);
            ConfigNode settings = null;
            foreach (ConfigNode node in GameDatabase.Instance.GetConfigNodes("AEROGUI"))
            {
                settings = node;
                break;
            }
            if (settings.HasValue("key"))
                key = (KeyCode)Enum.Parse(typeof(KeyCode), settings.GetValue("key"));
            if (settings.HasValue("useModifier"))
                bool.TryParse(settings.GetValue("useModifier"), out useModifier);
            if (settings.HasValue("WinterOwlMode"))
            {
                bool btmp;
                if (bool.TryParse(settings.GetValue("WinterOwlMode"), out btmp))
                    winterOwlModeOff = !btmp;
            }
        }

        public void OnGUI()
        {
            if (guiEnabled)
                windowPos = GUILayout.Window("AeroGUI".GetHashCode(), windowPos, DrawWindow, "AeroGUI");
            if (showThermal)
                windowPosThermal = GUILayout.Window("AeroGUIThermal".GetHashCode(), windowPosThermal, DrawWindowThermal, "Thermal Info");

        }
        public void Update()
        {
            // toggle GUI
            if (GameSettings.MODIFIER_KEY.GetKey() && Input.GetKeyDown(key))
                guiEnabled = !guiEnabled;

        }

        public void FixedUpdate()
        {
            if (HighLogic.LoadedSceneIsFlight && guiEnabled)
            {
                // clear values
                terminalV = alpha = mach = thrust = soundSpeed = climbrate = eas = lift = drag = lidForce = dragUpForce = pLift = pDrag = liftUp = grav = ldRatio = Q = pressure = density = ambientTemp = shockTemp = 0;
                // clear solar values
                solarFlux = bodyAlbedoFlux = bodyEmissiveFlux = bodySunFlux = effectiveFaceTemp = bodyTemperature = sunDot = atmosphereTemperatureOffset = altTempMult = latitude = latTempMod = axialTempMod = solarAMMult = finalAtmoMod = sunFinalMult = diurnalRange = 0d;
                backgroundRadTemp = PhysicsGlobals.SpaceTemperature;
                if ((object)FlightGlobals.ActiveVessel != null)
                {
                    // get non-air-dependent stats
                    Vessel v = FlightGlobals.ActiveVessel;
                    grav = v.GetTotalMass() * FlightGlobals.getGeeForceAtPosition(v.CoM).magnitude; // force of gravity
                    Vector3d nVel = v.srf_velocity.normalized;
                    alpha = Vector3d.Dot(v.transform.forward, nVel);
                    alpha = Math.Asin(alpha) * RAD2DEG; // angle of attack re: velocity
                    sideslip = Vector3d.Dot(v.transform.up, Vector3d.Exclude(v.transform.forward, nVel).normalized);
                    sideslip = Math.Acos(sideslip) * RAD2DEG; // sideslip re: velocity
                    if (double.IsNaN(sideslip))
                        sideslip = 0;
                    if (sideslip < 0)
                        sideslip = 180 + sideslip;
                    climbrate = Vector3d.Dot(v.srf_velocity, v.upAxis); // vertical speed in m/s
                    ambientTemp = v.atmosphericTemperature;
                    shockTemp = v.externalTemperature;
                    if (FlightGlobals.ActiveVessel.atmDensity > 0) // if we have air, get aero stats
                        GetAeroStats(nVel);

                    GetThermalStats(v);
                }
            }
        }

        public void GetThermalStats(Vessel vessel)
        {
            shockTemp = vessel.externalTemperature;
            double densityThermalLerp = 1d - vessel.atmDensity;
            if (densityThermalLerp < 0.5d)
            {
                densityThermalLerp = 0.25d / vessel.atmDensity;
            }
            backgroundRadTemp = UtilMath.Lerp(
                vessel.externalTemperature,
                PhysicsGlobals.SpaceTemperature,
                densityThermalLerp);
            // sun
            inSun = vessel.directSunlight;
            Vector3 sunVector = (FlightGlobals.Bodies[0].scaledBody.transform.position - ScaledSpace.LocalToScaledSpace(vessel.transform.position)).normalized;
            solarFlux = vessel.solarFlux;
            if (FlightGlobals.Bodies[0] != vessel.mainBody)
            {
                double sunDistanceSqr = ((FlightGlobals.Bodies[0].scaledBody.transform.position - vessel.mainBody.scaledBody.transform.position) * ScaledSpace.ScaleFactor).sqrMagnitude;
                bodySunFlux = PhysicsGlobals.SolarLuminosity / (4d * Math.PI * sunDistanceSqr);
                sunDot = Vector3.Dot(sunVector, vessel.upAxis);
                Vector3 mainBodyUp = vessel.mainBody.bodyTransform.up;
                float sunAxialDot = Vector3.Dot(sunVector, mainBodyUp);
                double bodyPolarAngle = Math.Acos(Vector3.Dot(mainBodyUp, vessel.upAxis));
                double sunPolarAngle = Math.Acos(sunAxialDot);
                double sunBodyMaxDot = (1d + Math.Cos(sunPolarAngle - bodyPolarAngle)) * 0.5d;
                double sunBodyMinDot = (1d + Math.Cos(sunPolarAngle + bodyPolarAngle)) * 0.5d;

                double fracBodyPolar = bodyPolarAngle;
                double fracSunPolar = sunPolarAngle;

                if (bodyPolarAngle > Math.PI * 0.5d)
                {
                    fracBodyPolar = Math.PI - fracBodyPolar;
                    fracSunPolar = Math.PI - fracSunPolar;
                }

                double bodyDayFraction = (Math.PI * 0.5d + Math.Asin((Math.PI * 0.5d - fracSunPolar) / fracBodyPolar)) * (1d / Math.PI);
                // correct sunDot based on the fact peak temp is ~3pm and min temp is ~3am, or so we will assume
                double sunDotCorrected = (1d + Vector3.Dot(sunVector, Quaternion.AngleAxis(-45f * Mathf.Sign((float)vessel.mainBody.rotationPeriod), mainBodyUp) * vessel.upAxis)) * 0.5d;

                // get normalized dot
                double sunDotNormalized = (sunDotCorrected - sunBodyMinDot) / (sunBodyMaxDot - sunBodyMinDot);
                if (double.IsNaN(sunDotNormalized))
                {
                    if (sunDotCorrected > 0.5d)
                        sunDotNormalized = 1d;
                    else
                        sunDotNormalized = 0d;
                }

                // get latitude-based changes, if body has an atmosphere
                if (vessel.mainBody.atmosphere)
                {
                    float latitude = (float)(Math.PI * 0.5d - fracBodyPolar);
                    latitude *= Mathf.Rad2Deg;
                    diurnalRange = vessel.mainBody.latitudeTemperatureSunMultCurve.Evaluate(latitude);
                    latTempMod = vessel.mainBody.latitudeTemperatureBiasCurve.Evaluate(latitude);
                    axialTempMod = vessel.mainBody.axialTemperatureSunMultCurve.Evaluate(sunAxialDot);
                    atmosphereTemperatureOffset = latTempMod + diurnalRange * sunDotNormalized + axialTempMod;
                    altTempMult = vessel.mainBody.atmosphereTemperatureSunMultCurve.Evaluate((float)vessel.altitude);

                    // calculate air mass etc
                    double depthFactor = 1.0;

                    if (vessel.atmDensity > 0)
                    {
                        // get final temp mod
                        finalAtmoMod = atmosphereTemperatureOffset * altTempMult;

                        // get solar air stuff
                        double rcosz = vessel.mainBody.radiusAtmoFactor * sunDot;
                        depthFactor = vessel.mainBody.GetSolarPowerFactor(vessel.atmDensity);
                        // alternative 1: absolute value
                        /*if(rcosz < 0)
                        {
                            rcosz = -rcosz;
                        }
                        solarAMD = Math.Sqrt(rcosz * rcosz + 2 * radiusFactor + 1) - rcosz;*/

                        // alternative 2: clamp
                        if (rcosz < 0)
                            solarAMMult = 1 / Math.Sqrt(2 * vessel.mainBody.radiusAtmoFactor + 1);
                        else
                            solarAMMult = 1 / (Math.Sqrt(rcosz * rcosz + 2 * vessel.mainBody.radiusAtmoFactor + 1) - rcosz);

                        sunFinalMult = depthFactor * solarAMMult;


                        // get hypersonic convective shock temp
                        double machLerp = (vessel.mach - PhysicsGlobals.MachConvectionStart) / (PhysicsGlobals.MachConvectionEnd - PhysicsGlobals.MachConvectionStart);
                        if (machLerp > 0)
                        {
                            machLerp = Math.Pow(machLerp, PhysicsGlobals.MachConvectionExponent);
                            machLerp = Math.Min(1d, machLerp);
                            double machExtTemp = Math.Pow(0.5d * vessel.convectiveMachFlux
                                / (PhysicsGlobals.StefanBoltzmanConstant * PhysicsGlobals.RadiationFactor), 1d / PhysicsGlobals.PartEmissivityExponent);
                            shockTemp = Math.Max(shockTemp, UtilMath.LerpUnclamped(shockTemp, machExtTemp, machLerp));
                        }
                    }
                }

                // now body properties
                // Now calculate albedo and emissive fluxes, and body temperature under vessel
                double nightTempScalar = 0d;
                double bodyMinTemp = 0d;
                double bodyMaxTemp = 0d;
                if (vessel.mainBody.atmosphere)
                {
                    double baseTemp = vessel.mainBody.GetTemperature(0d);
                    bodyTemperature = baseTemp + atmosphereTemperatureOffset;
                    bodyMinTemp = baseTemp + (vessel.mainBody.latitudeTemperatureBiasCurve.Evaluate(90f) // no lat sun mult since night
                        + vessel.mainBody.axialTemperatureSunMultCurve.Evaluate(-(float)vessel.mainBody.orbit.inclination));
                    bodyMaxTemp = baseTemp + (vessel.mainBody.latitudeTemperatureBiasCurve.Evaluate(0f)
                        + (vessel.mainBody.latitudeTemperatureSunMultCurve.Evaluate(0f))
                        + vessel.mainBody.axialTemperatureSunMultCurve.Evaluate((float)vessel.mainBody.orbit.inclination));

                    nightTempScalar = 1d - Math.Sqrt(bodyMaxTemp) * 0.0016d;
                    nightTempScalar = UtilMath.Clamp01(nightTempScalar);
                }
                else
                {
                    double spaceTemp4 = PhysicsGlobals.SpaceTemperature;
                    spaceTemp4 *= spaceTemp4;
                    spaceTemp4 *= spaceTemp4;

                    // now calculate two temperatures: the effective temp, and the maximum possible surface temperature
                    double sbERecip = 1d / (PhysicsGlobals.StefanBoltzmanConstant * vessel.mainBody.emissivity);
                    double tmp = bodySunFlux * (1d - vessel.mainBody.albedo) * sbERecip;
                    double effectiveTemp = Math.Pow(0.25d * tmp + spaceTemp4, 0.25d);
                    double fullSunTemp = Math.Pow(tmp + spaceTemp4, 0.25d);

                    // now use some magic numbers to calculate minimum and maximum temperatures
                    double tempOffset = fullSunTemp - effectiveTemp;
                    bodyMaxTemp = effectiveTemp + Math.Sqrt(tempOffset) * 2d;
                    bodyMinTemp = effectiveTemp - Math.Pow(tempOffset, 1.1d) * 1.22d;

                    double lerpVal = 2d / Math.Sqrt(Math.Sqrt(vessel.mainBody.solarDayLength));
                    bodyMaxTemp = UtilMath.Lerp(bodyMaxTemp, effectiveTemp, lerpVal);
                    bodyMinTemp = UtilMath.Lerp(bodyMinTemp, effectiveTemp, lerpVal);

                    double latitudeLerpVal = Math.Max(0d, sunBodyMaxDot * 2d - 1d);
                    latitudeLerpVal = Math.Sqrt(latitudeLerpVal);
                    nightTempScalar = 1d - Math.Sqrt(bodyMaxTemp) * 0.0016d;
                    nightTempScalar = UtilMath.Clamp01(nightTempScalar);
                    double tempDiff = (bodyMaxTemp - bodyMinTemp) * latitudeLerpVal;
                    double dayTemp = bodyMinTemp + tempDiff;
                    double nightTemp = bodyMinTemp + tempDiff * nightTempScalar;
                    bodyTemperature = Math.Max(PhysicsGlobals.SpaceTemperature,
                        nightTemp + (dayTemp - nightTemp) * sunDotNormalized + vessel.mainBody.coreTemperatureOffset);
                }

                // Quite a lerp. We need to get the lerp between front and back faces, then lerp between that and the top facing
                effectiveFaceTemp = UtilMath.LerpUnclamped(
                    UtilMath.LerpUnclamped( // horizontal component
                        UtilMath.LerpUnclamped(bodyMinTemp, bodyMaxTemp, // front face
                            UtilMath.LerpUnclamped(bodyEmissiveScalarS0Front, bodyEmissiveScalarS1, nightTempScalar)),
                        UtilMath.LerpUnclamped(bodyMinTemp, bodyMaxTemp, // back face
                            UtilMath.LerpUnclamped(bodyEmissiveScalarS0Back, bodyEmissiveScalarS1, nightTempScalar)),
                        sunDotNormalized),
                    UtilMath.LerpUnclamped(bodyMinTemp, bodyMaxTemp, UtilMath.LerpUnclamped(bodyEmissiveScalarS0Top, bodyEmissiveScalarS1Top, nightTempScalar)),
                    sunBodyMaxDot);

                double temp4 = UtilMath.Lerp(bodyTemperature, effectiveFaceTemp, 0.2d + vessel.altitude / vessel.mainBody.Radius * 0.5d);
                temp4 *= temp4;
                temp4 *= temp4;
                double bodyFluxScalar = (4d * Math.PI * vessel.mainBody.Radius * vessel.mainBody.Radius)
                    / (4d * Math.PI * (vessel.mainBody.Radius + vessel.altitude) * (vessel.mainBody.Radius + vessel.altitude));
                bodyEmissiveFlux = PhysicsGlobals.StefanBoltzmanConstant * vessel.mainBody.emissivity * temp4 * bodyFluxScalar;
                bodyAlbedoFlux = bodySunFlux * 0.5d * (sunDot + 1f) * vessel.mainBody.albedo * bodyFluxScalar;

                // density lerps
                bodyEmissiveFlux = UtilMath.Lerp(0d, bodyEmissiveFlux, densityThermalLerp);
                bodyAlbedoFlux = UtilMath.Lerp(0d, bodyAlbedoFlux, densityThermalLerp);
            }
        }
            

            

        // This method will calculate various useful stats
        // That we display in the GUI.
        public void GetAeroStats(Vector3d nVel)
        {
            Vessel v = FlightGlobals.ActiveVessel;
            Vector3d vLift = Vector3d.zero; // the sum of lift from all parts
            Vector3d vDrag = Vector3d.zero; // the sum of drag from all parts
            double sqrMag = v.srf_velocity.sqrMagnitude;
            Q = 0.5 * v.atmDensity * sqrMag; // dynamic pressure, aka Q
            eas = Math.Sqrt(sqrMag * v.atmDensity / 1.225); // Equivalent Air Speed
            // i.e. your airspeed at sea level with the same Q
            density = v.atmDensity;
            pressure = v.staticPressurekPa * 1000.0;
            mach = v.rootPart.machNumber;
            soundSpeed = v.speedOfSound;
            double dTime = TimeWarp.fixedDeltaTime;

            // Now we loop through all parts, checking the modules in each part
            // This way we get all drag, lift, and thrust.
            for (int i = 0; i < v.Parts.Count; ++i)
            {
                Part p = v.Parts[i];

                // get part drag (but not wing/surface drag)
                vDrag += -p.dragVectorDir * p.dragScalar;
                if (!p.hasLiftModule)
                {
                    Vector3 bodyLift = p.transform.rotation * (p.bodyLiftScalar * p.DragCubes.LiftForce);
                    bodyLift = Vector3.ProjectOnPlane(bodyLift, -p.dragVectorDir);
                    vLift += bodyLift;
                }

                // get convection
                convectiveTotal += p.thermalConvectionFlux * dTime;

                // now find modules
                for (int j = 0; j < p.Modules.Count; ++j)
                {
                    var m = p.Modules[j];
                    if (m is ModuleLiftingSurface) // control surface derives from this
                    {
                        ModuleLiftingSurface wing = (ModuleLiftingSurface)m;
                        vLift += wing.liftForce;
                        vDrag += wing.dragForce;
                    }
                    // Get thrust
                    if (m is ModuleEngines) // FX derives from this
                    {
                        thrust += ((ModuleEngines)m).finalThrust;
                    }
                }
            }
            // pLift is 'pure' lift, and same for drag, i.e. just the magnitude of each
            pLift = vLift.magnitude;
            pDrag = vDrag.magnitude;
            Vector3d force = vLift + vDrag; // sum of all forces on the craft
            Vector3d liftDir = -Vector3d.Cross(v.transform.right, nVel); // we need the "lift" direction, which
            // is "up" from our current velocity vector and roll angle.

            // Now we can compute the dots.
            lift = Vector3d.Dot(force, liftDir); // just the force in the 'lift' direction
            liftUp = Vector3d.Dot(force, v.upAxis); // just the force in the 'up' direction (note, some of it may be drag!)
            drag = Vector3d.Dot(force, -nVel); // drag force, = pDrag + lift-induced drag
            lidForce = Vector3d.Dot(vLift, -nVel); // Lift Induced Drag
            dragUpForce = Vector3d.Dot(vDrag, v.upAxis); // any drag in the opposite direction of gravity
            ldRatio = lift / drag; // Lift / Drag ratio
            terminalV = Math.Sqrt(grav / drag) * v.speed;
            if (double.IsNaN(terminalV))
                terminalV = 0d;
        }


        public void DrawWindow(int windowID)
        {
            // Enable closing of the window tih "x"
            GUIStyle buttonStyle = new GUIStyle(GUI.skin.button);
            buttonStyle.padding = new RectOffset(5, 5, 3, 0);
            buttonStyle.margin = new RectOffset(1, 1, 1, 1);
            buttonStyle.stretchWidth = false;
            buttonStyle.stretchHeight = false;
            GUIStyle labelStyle = new GUIStyle(GUI.skin.label);
            labelStyle.wordWrap = false;

            GUILayout.BeginVertical();

            GUILayout.FlexibleSpace();

            GUILayout.BeginHorizontal();
            if (GUILayout.Button("X", buttonStyle))
            {
                guiEnabled = false;
            }
            GUILayout.EndHorizontal();

            string tmp;

            // Now the basic craft stats
            // see above in the method where they are calculated for explanations.


            GUILayout.BeginHorizontal();
            GUILayout.Label("Mach: " + mach.ToString("N3"), labelStyle);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("EAS: " + eas.ToString("N2") + " m/s", labelStyle);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Terminal Vel (est): " + terminalV.ToString("N1") + " m/s", labelStyle);
            GUILayout.EndHorizontal();

            if (winterOwlModeOff)
            {
                GUILayout.BeginHorizontal();
                GUILayout.Label(" ------ ", labelStyle);
                GUILayout.EndHorizontal();

                tmp = (Q > 0d && Q < 0.1d) ? Q.ToString("E6") : Q.ToString("N1");
                GUILayout.BeginHorizontal();
                GUILayout.Label("Dynamic Pressure: " + tmp + " Pa", labelStyle);
                GUILayout.EndHorizontal();

                tmp = (pressure > 0d && pressure < 0.1d) ? pressure.ToString("E6") : pressure.ToString("N1");
                GUILayout.BeginHorizontal();
                GUILayout.Label("Static Pressure: " + tmp + " Pa", labelStyle);
                GUILayout.EndHorizontal();

                tmp = (density > 0d && density < 1e-5d) ? density.ToString("E6") : density.ToString("N6");
                GUILayout.BeginHorizontal();
                GUILayout.Label("Density: " + tmp + " kg/m^3", labelStyle);
                GUILayout.EndHorizontal();

                /*double dSqrt = Math.Sqrt(density);
                tmp = (dSqrt > 0d && dSqrt < 1e-5d) ? dSqrt.ToString("E6") : dSqrt.ToString("N6");
                GUILayout.BeginHorizontal();
                GUILayout.Label("Density (Sqrt): " + tmp, labelStyle);
                GUILayout.EndHorizontal();*/

                GUILayout.BeginHorizontal();
                GUILayout.Label("Static Amb. Temp: " + ambientTemp.ToString("N2") + " K", labelStyle);
                GUILayout.EndHorizontal();

                GUILayout.BeginHorizontal();
                GUILayout.Label("External Temp: " + shockTemp.ToString("N2") + " K", labelStyle);
                GUILayout.EndHorizontal();

                GUILayout.BeginHorizontal();
                GUILayout.Label("Speed of sound: " + soundSpeed.ToString("N2") + " m/s", labelStyle);
                GUILayout.EndHorizontal();

                GUILayout.BeginHorizontal();
                GUILayout.Label(" ------ ", labelStyle);
                GUILayout.EndHorizontal();

                // Now lift/drag related stats
                GUILayout.BeginHorizontal();
                GUILayout.Label("AoA: " + alpha.ToString("N3") + " deg", labelStyle);
                GUILayout.EndHorizontal();

                GUILayout.BeginHorizontal();
                GUILayout.Label("Sideslip: " + sideslip.ToString("N3") + " deg", labelStyle);
                GUILayout.EndHorizontal();

                GUILayout.BeginHorizontal();
                GUILayout.Label("Climb rate: " + climbrate.ToString("N3") + " m/s", labelStyle);
                GUILayout.EndHorizontal();

                GUILayout.BeginHorizontal();
                GUILayout.Label("Total Lift: " + lift.ToString("N3") + " kN", labelStyle);
                GUILayout.EndHorizontal();

                GUILayout.BeginHorizontal();
                GUILayout.Label("Total Drag: " + drag.ToString("N3") + " kN", labelStyle);
                GUILayout.EndHorizontal();

                GUILayout.BeginHorizontal();
                GUILayout.Label("Lift / Drag Ratio: " + ldRatio.ToString("N3"), labelStyle);
                GUILayout.EndHorizontal();

                GUILayout.BeginHorizontal();
                GUILayout.Label("Thrust: " + thrust.ToString("N3") + " kN", labelStyle);
                GUILayout.EndHorizontal();

                GUILayout.BeginHorizontal();
                GUILayout.Label("Grav Force (weight): " + grav.ToString("N3") + " kN", labelStyle);
                GUILayout.EndHorizontal();

                GUILayout.BeginHorizontal();
                GUILayout.Label("Upwards Force: " + liftUp.ToString("N3") + " kN", labelStyle);
                GUILayout.EndHorizontal();

                GUILayout.BeginHorizontal();
                GUILayout.Label("L-I-D: " + lidForce.ToString("N3") + " kN", labelStyle);
                GUILayout.EndHorizontal();

                GUILayout.BeginHorizontal();
                if (GUILayout.Button("Thermal", buttonStyle))
                {
                    showThermal = !showThermal;
                }
                GUILayout.EndHorizontal();
            }

            GUILayout.EndVertical();
            GUI.DragWindow();
        }

        public void DrawWindowThermal(int windowID)
        {
            // Enable closing of the window tih "x"
            GUIStyle buttonStyle = new GUIStyle(GUI.skin.button);
            buttonStyle.padding = new RectOffset(5, 5, 3, 0);
            buttonStyle.margin = new RectOffset(1, 1, 1, 1);
            buttonStyle.stretchWidth = false;
            buttonStyle.stretchHeight = false;
            GUIStyle labelStyle = new GUIStyle(GUI.skin.label);
            labelStyle.wordWrap = false;

            GUILayout.BeginVertical();

            GUILayout.FlexibleSpace();

            GUILayout.BeginHorizontal();
            if (GUILayout.Button("X", buttonStyle))
            {
                showThermal = false;
            }
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Solar Flux: " + (solarFlux * 0.001d).ToString("N3") + " kW", labelStyle);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Solar Flux Mult: " + sunFinalMult.ToString("N5"), labelStyle);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Air Mass Mult: " + solarAMMult.ToString("N5"), labelStyle);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Sun Incidence: " + sunDot.ToString("N4"), labelStyle);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Latitude: " + latitude.ToString("N3"), labelStyle);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Lat. Temp Mod: " + latTempMod.ToString("N2") + " K", labelStyle);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Seasonal Temp Mod: " + axialTempMod.ToString("N2") + " K", labelStyle);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Diurnal Temp Range: " + diurnalRange.ToString("N2") + " K", labelStyle);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Alt Temp Offset Mult: " + altTempMult.ToString("N5"), labelStyle);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Final Temp Mod: " + finalAtmoMod.ToString("N2") + " K", labelStyle);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label(" ------ ", labelStyle);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Background Radiation Temp: " + backgroundRadTemp.ToString("N2") + " K", labelStyle);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Body Temp under craft: " + bodyTemperature .ToString("N2") + " K", labelStyle);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Effective Body Face Temp: " + effectiveFaceTemp.ToString("N2") + " K", labelStyle);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Body Radiation Flux: " + (bodyEmissiveFlux * 0.001d).ToString("N5") + " kW", labelStyle);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Body Albedo Flux: " + (bodyAlbedoFlux * 0.001d).ToString("N5") + " kW", labelStyle);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Convective Heat Load: " + convectiveTotal.ToString("N3") + " kJ", labelStyle);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Reset Conv. Counter", buttonStyle))
            {
                convectiveTotal = 0d;
            }
            GUILayout.EndHorizontal();


            GUILayout.EndVertical();
            GUI.DragWindow();
        }
    }
}
