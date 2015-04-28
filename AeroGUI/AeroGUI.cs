/*by NathanKell
 * License: 
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
        public static bool showThermal = false;

        // Solar bits
        double scaleHeightMult, sunDot, latitudeDot, latTempMod, latTempMult, solarAMMult, sunTempMod, sunFinalMult, diurnalRange;
        bool inSun;

        // display forces
        public double terminalV, alpha, sideslip, soundSpeed, mach, eas, thrust, climbrate, lift, drag, lidForce, dragUpForce, pLift, pDrag, liftUp, grav, ldRatio, Q, pressure, density, ambientTemp, shockTemp;

        // thermal counter
        public double convectiveTotal = 0d;

        public void Start()
        {
            enabled = true;
            scaleHeightMult = -Math.Log(0.000001);
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
                diurnalRange = sunDot = latitudeDot = latTempMod = latTempMult = solarAMMult = sunTempMod = sunFinalMult = 0;
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

                    GetSunStats(v);
                }
            }
        }

        public void GetSunStats(Vessel vessel)
        {
            inSun = vessel.directSunlight;
            Vector3 sunVector = (FlightGlobals.Bodies[0].scaledBody.transform.position - ScaledSpace.LocalToScaledSpace(vessel.transform.position)).normalized;
            sunDot = Vector3.Dot(sunVector, vessel.upAxis);
            double depthFactor = 1.0;
            solarAMMult = 1.0;
            if (vessel.atmDensity > 0)
            {
                double radiusFactor = vessel.mainBody.Radius / vessel.mainBody.atmosphereDepth * scaleHeightMult;
                double rcosz = radiusFactor * sunDot;
                depthFactor = vessel.mainBody.GetSolarPowerFactor(vessel.atmDensity);
                // alternative 1: absolute value
                /*if(rcosz < 0)
                {
                    rcosz = -rcosz;
                }
                solarAMD = Math.Sqrt(rcosz * rcosz + 2 * radiusFactor + 1) - rcosz;*/

                // alternative 2: clamp
                if (rcosz < 0)
                    solarAMMult = 1 / Math.Sqrt(2 * radiusFactor + 1);
                else
                    solarAMMult = 1 / (Math.Sqrt(rcosz * rcosz + 2 * radiusFactor + 1) - rcosz);
            }
            sunFinalMult = depthFactor * solarAMMult;
            latitudeDot = Math.Abs(Vector3.Dot(vessel.mainBody.bodyTransform.up, vessel.upAxis));
            latTempMod = 15d - (75d * latitudeDot) * 0.5d; // extra 0.5 because dot never gets very high at the poles
            latTempMult = (10d + 8d * latitudeDot);
            sunTempMod = latTempMod + latTempMult * (1 + sunDot) * 0.5d; // normalized dot
            double polarAng = Math.Acos(Vector3.Dot(vessel.mainBody.bodyTransform.up, vessel.upAxis));
            if (polarAng < Math.PI * 0.5) // <90deg
            {
                double sunPoleAngle = Math.Acos(Vector3.Dot(vessel.mainBody.bodyTransform.up, sunVector));
                double dayDot = (1 + Math.Cos(sunPoleAngle - polarAng)) * .5;
                double nightDot = (1 + Math.Cos(sunPoleAngle + polarAng)) * .5;
                diurnalRange = (dayDot - nightDot) * latTempMult;
            }
            else
            {
                double sunPoleAngle = Math.Acos(Vector3.Dot(-vessel.mainBody.bodyTransform.up, sunVector));
                double dayDot = (1 + Math.Cos(sunPoleAngle - (Math.PI - polarAng))) * .5;
                double nightDot = (1 + Math.Cos(sunPoleAngle + (Math.PI - polarAng))) * .5;
                diurnalRange = (dayDot - nightDot) * latTempMult;
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

            double dSqrt = Math.Sqrt(density);
            tmp = (dSqrt > 0d && dSqrt < 1e-5d) ? dSqrt.ToString("E6") : dSqrt.ToString("N6");
            GUILayout.BeginHorizontal();
            GUILayout.Label("Density (Sqrt): " + tmp, labelStyle);
            GUILayout.EndHorizontal();

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

            GUILayout.BeginHorizontal();
            GUILayout.Label("Mach: " + mach.ToString("N3"), labelStyle);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("EAS: " + eas.ToString("N2") + " m/s", labelStyle);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Terminal Vel (est): " + terminalV.ToString("N1") + " m/s", labelStyle);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Climb rate: " + climbrate.ToString("N3") + " m/s", labelStyle);
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

            /*GUILayout.BeginHorizontal();
            if (GUILayout.Button("Thermal", buttonStyle))
            {
                showThermal = !showThermal;
            }
            if (GUILayout.Button("Reset Conv. Counter", buttonStyle))
            {
                convectiveTotal = 0d;
            }
            GUILayout.EndHorizontal();*/

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
            GUILayout.Label("In Sunlight: " + inSun.ToString(), labelStyle);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Solar Flux Mult: " + sunFinalMult.ToString("N5"), labelStyle);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Air Mass Mult: " + solarAMMult.ToString("N5"), labelStyle);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Ext Temp Mod: " + sunTempMod.ToString("N2") + " K", labelStyle);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Sun Incidence" + sunDot.ToString("N4"), labelStyle);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Latitude: " + (Math.Asin(latitudeDot) * RAD2DEG).ToString("N3"), labelStyle);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Lat. Temp Mod: " + latTempMod.ToString("N2") + " K", labelStyle);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Diurnal Temp Mult: " + latTempMult.ToString("N2") + " K", labelStyle);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Diurnal Temp Range: " + diurnalRange.ToString("N2") + " K", labelStyle);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label(" ------ ", labelStyle);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Heat Load: " + convectiveTotal + " kJ", labelStyle);
            GUILayout.EndHorizontal();


            GUILayout.EndVertical();
            GUI.DragWindow();
        }
    }
}
