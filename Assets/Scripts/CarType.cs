using System.Collections.Generic;
using UnityEngine;
using System.Linq;

namespace CPH
{
    public class CarType : MonoBehaviour
    {
        public float b = 1.4f;
        public float c = 1.6f;
        public float length = 5f;
        public float height = 1.4f;
        [HideInInspector]
        public float wheelbase;
        public float mass = 1289f;
        public Vector3 inertia = Vector3.zero;
        public float track = 2f;
        public float wheelradius = 0.41f;
        public float drivetrainLoss = 0.1f;
        public float[] gks = { 4.8f, 4.7f, 2.99f, 2.15f, 1.77f, 1.52f, 1.28f }; // Gear ratios
        public float driveRatio = 3.35f;
        [HideInInspector]
        public int minRpm;
        [HideInInspector]
        public int redLine;
        public Dictionary<int, float> torqueCurve = new Dictionary<int, float>
        {
            { 1000, 250 },
            { 2000, 425 },
            { 4500, 525 },
            { 7000, 450 },
            { 7500, 375 },
        };
        List<int> curveKeys;
        public Vector3 COM = Vector3.zero;
        public float SS_F = 250000; // Spring stiffness
        public float SS_R = 250000; // Spring stiffness
        public float DS_F = 25000; // Damper stiffness
        public float DS_R = 25000; // Damper stiffness
        public float restLength_F = 0.5f;
        public float restLength_R = 0.5f;
        public float springTravel = 0.2f;
        public float sphereCastRatio = 0.45f;
        public float max_lat_wt = 1300;
        public float max_long_wt = 13000;
        public float Cdrag = 0.12f;
        public float Crr = 3f;
        public float CS_R = 5.2f; // Cornering stiffness
        public float CS_F = 5.0f; // Cornering stiffness
        public float Cbrake = 20000;
        public float maxGrip = 4.0f;
        public float maxSteerAngle = Mathf.PI / 4.0f;
        public float long_V_activate_thres = 0.7f;
        public float rayOffsetZ_F = 1.6f;
        public float rayOffset_T_F = 0.12f; // Relative to track
        public float rayOffsetZ_R = 1.4f;
        public float rayOffset_T_R = 0.12f; // Relative to track
        public int suspension_activate_deg = 12;
        public float weak_suspension_coef = 0.05f;
        public float angular_friction = Mathf.PI / 128;
        public Transform[] wheels;
        public Transform[] wheelMeshes;

        public void setUp()
        {
            wheelbase = b + c;
            curveKeys = torqueCurve.Keys.ToList();
            minRpm = curveKeys.First();
            redLine = curveKeys.Last();
        }

        public float GetTorqueAtRPM(int rpm)
        {
            // If RPM is below the lowest or above the highest, return boundary values
            if (rpm <= minRpm) return torqueCurve[minRpm];
            if (rpm >= redLine) return torqueCurve[redLine];

            // Find two closest RPM points
            for (int i = 0; i < curveKeys.Count - 1; i++)
            {
                if (rpm >= curveKeys[i] && rpm <= curveKeys[i + 1])
                {
                    // Apply linear interpolation formula
                    float T1 = torqueCurve[curveKeys[i]];
                    float T2 = torqueCurve[curveKeys[i + 1]];
                    int RPM1 = curveKeys[i];
                    int RPM2 = curveKeys[i + 1];

                    return T1 + ((T2 - T1) * (rpm - RPM1) / (RPM2 - RPM1));
                }
            }

            return 0; // Should never reach here
        }
    }
}
