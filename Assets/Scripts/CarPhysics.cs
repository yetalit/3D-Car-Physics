using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace CPH
{
    public class CarPhysics : MonoBehaviour
    {
        public Car userCar;

        public float throttleRate = 0.003f;
        public float steeringRate = Mathf.PI / 1024;

        public RectTransform[] uiNeedles;

        private void Start()
        {
            userCar.setUp(userCar.transform.position, userCar.transform.rotation);
        }

        private void OnGUI()
        {
            GUI.skin.label.fontSize = Screen.width/70;
            // Display stats on the screen
            GUI.Label(new Rect(10, 10, 800, 50), "Throttle: " + userCar.throttle.ToString());
            GUI.Label(new Rect(10, 60, 800, 50), "Steering Angle: " + userCar.steerangle.ToString());
            GUI.Label(new Rect(10, 110, 800, 50), "Gear: " + userCar.gear.ToString());
        }

        private void Update()
        {
            //userCar.transform.position = userCar.position_wc;
            //userCar.transform.rotation = userCar.rotation;

            userCar.cartype.wheels[0].localRotation = Quaternion.Euler(userCar.cartype.wheels[0].localRotation.eulerAngles.x, Mathf.Rad2Deg * userCar.steerangle, userCar.cartype.wheels[0].localRotation.eulerAngles.z);
            userCar.cartype.wheels[1].localRotation = Quaternion.Euler(userCar.cartype.wheels[1].localRotation.eulerAngles.x, Mathf.Rad2Deg * userCar.steerangle, userCar.cartype.wheels[1].localRotation.eulerAngles.z);

            if (userCar.isGrounded)
            {
                userCar.cartype.wheelMeshes[0].Rotate(Vector3.right, userCar.wheelRot * Time.deltaTime, Space.Self);
                userCar.cartype.wheelMeshes[1].Rotate(Vector3.right, userCar.wheelRot * Time.deltaTime, Space.Self);
                userCar.cartype.wheelMeshes[2].Rotate(Vector3.right, userCar.wheelRot * Time.deltaTime, Space.Self);
                userCar.cartype.wheelMeshes[3].Rotate(Vector3.right, userCar.wheelRot * Time.deltaTime, Space.Self);
            }

            userCar.GetComponent<AudioSource>().pitch = userCar.rpm * 3.0f / userCar.cartype.redLine;

            uiNeedles[0].rotation = Quaternion.Euler(new Vector3(0, 0, -142 - userCar.rpm * 0.028f));
            uiNeedles[1].rotation = Quaternion.Euler(new Vector3(0, 0, -142 - 2.237f * userCar.velocity_wc.magnitude * 1.4f));

            if (Input.GetKey(KeyCode.Return)) // Restart
            {
                SceneManager.LoadScene(0);
            }

            if (Input.GetKey(KeyCode.Mouse0)) // Throttle up
            {
                userCar.throttle += throttleRate;
            }
            else if (Input.GetKey(KeyCode.Mouse1)) // Throttle down
            {
                userCar.throttle -= throttleRate;
            }
            userCar.throttle = Mathf.Clamp(userCar.throttle, -1f, 1f);

            // Brake
            if (Input.GetKey(KeyCode.Mouse2)) // brake
            {
                userCar.brake = 1;
                //car.throttle = 0;
            }
            else
            {
                userCar.brake = 0;
            }

            // Steering
            if (Input.GetKey(KeyCode.A)) // steer left
            {
                userCar.steerangle -= steeringRate;
                if (userCar.steerangle < -userCar.cartype.maxSteerAngle)
                {
                    userCar.steerangle = -userCar.cartype.maxSteerAngle;
                }
            }
            else if (Input.GetKey(KeyCode.D)) // steer right
            {
                userCar.steerangle += steeringRate;
                if (userCar.steerangle > userCar.cartype.maxSteerAngle)
                {
                    userCar.steerangle = userCar.cartype.maxSteerAngle;
                }
            }
            else
            {
                if (userCar.steerangle > 0)
                {
                    if (userCar.steerangle >= steeringRate)
                        userCar.steerangle -= steeringRate;
                    else
                    {
                        userCar.steerangle = 0;
                    }
                }
                else if (userCar.steerangle < 0)
                {
                    if (userCar.steerangle <= -steeringRate)
                        userCar.steerangle += steeringRate;
                    else
                    {
                        userCar.steerangle = 0;
                    }
                }
            }

            userCar.rearSlip = false;
            userCar.frontSlip = false;
            if (Input.GetKey(KeyCode.S)) // rear axle slip
            {
                userCar.rearSlip = true;
            }
            if (Input.GetKey(KeyCode.Space)) // both axles slip
            {
                userCar.frontSlip = true;
                userCar.rearSlip = true;
            }
        }

        private void FixedUpdate()
        {
            DoPhysics(userCar, Time.fixedDeltaTime);
        }

        private void DoPhysics(Car car, float delta_t)
        {
            car.velocity_wc = car.rb.linearVelocity;
            car.angularVelocity = car.rb.angularVelocity;
            car.position_wc = car.rb.position;
            car.rotation = car.rb.rotation;
            car.cartype.inertia = (1 / 12f) * car.cartype.mass * new Vector3(car.cartype.length * car.cartype.length + car.cartype.height * car.cartype.height, car.cartype.length * car.cartype.length + car.cartype.track * car.cartype.track, car.cartype.track * car.cartype.track + car.cartype.height * car.cartype.height);
            car.cartype.inertia += car.cartype.mass * new Vector3(car.cartype.COM.y * car.cartype.COM.y + car.cartype.COM.z * car.cartype.COM.z, car.cartype.COM.x * car.cartype.COM.x + car.cartype.COM.z * car.cartype.COM.z, car.cartype.COM.x * car.cartype.COM.x + car.cartype.COM.y * car.cartype.COM.y);
            car.rb.centerOfMass = car.cartype.COM;
            car.rb.inertiaTensor = car.cartype.inertia;

            // Transform velocity from world reference frame to car reference frame
            Vector3 velocity = Utils.InverseTransformDirection(car.rotation, car.velocity_wc);

            Vector2 slip_angle_f = Vector2.zero;
            Vector2 slip_angle_r = Vector2.zero;

            if (Mathf.Abs(velocity.z) > car.cartype.long_V_activate_thres)
            {
                slip_angle_f.x = car.steerangle * Mathf.Sign(velocity.z) - Mathf.Atan((velocity.x + car.angularVelocity.y * car.cartype.b) / (Mathf.Abs(velocity.z) - car.cartype.track * car.angularVelocity.y / 2));
                slip_angle_f.y = car.steerangle * Mathf.Sign(velocity.z) - Mathf.Atan((velocity.x + car.angularVelocity.y * car.cartype.b) / (Mathf.Abs(velocity.z) + car.cartype.track * car.angularVelocity.y / 2));
                slip_angle_r.x = -Mathf.Atan((velocity.x - car.angularVelocity.y * car.cartype.c) / (Mathf.Abs(velocity.z) - car.cartype.track * car.angularVelocity.y / 2));
                slip_angle_r.y = -Mathf.Atan((velocity.x - car.angularVelocity.y * car.cartype.c) / (Mathf.Abs(velocity.z) + car.cartype.track * car.angularVelocity.y / 2));
            }

            Vector2 F_lat_f = car.cartype.CS_F * slip_angle_f;
            F_lat_f.x = Mathf.Clamp(F_lat_f.x, -car.cartype.maxGrip, car.cartype.maxGrip);
            F_lat_f.y = Mathf.Clamp(F_lat_f.y, -car.cartype.maxGrip, car.cartype.maxGrip);
            Vector2 F_lat_r = car.cartype.CS_R * slip_angle_r;
            F_lat_r.x = Mathf.Clamp(F_lat_r.x, -car.cartype.maxGrip, car.cartype.maxGrip);
            F_lat_r.y = Mathf.Clamp(F_lat_r.y, -car.cartype.maxGrip, car.cartype.maxGrip);

            float[] load = new float[4]; // FL, FR, RL, RR
            load[0] = load[1] = (car.cartype.c / car.cartype.wheelbase) * car.cartype.mass * 9.8f / 2;
            load[2] = load[3] = (car.cartype.b / car.cartype.wheelbase) * car.cartype.mass * 9.8f / 2;

            F_lat_f *= load[0];
            F_lat_r *= load[2];

            if (car.rearSlip)
                F_lat_r *= 0.5f;
            if (car.frontSlip)
                F_lat_f *= 0.5f;

            if (car.isGrounded)
            {
                car.rpm = (int)(Mathf.Abs(velocity.z) * 60 * car.cartype.gks[car.gear] * car.cartype.driveRatio / (2 * Mathf.PI * car.cartype.wheelradius));
            }
            car.rpm = Mathf.Clamp(car.rpm, car.cartype.minRpm, car.cartype.redLine);
            // Automatic transmission
            if (car.isGrounded)
            {
                if (car.throttle >= 0)
                {
                    if (car.gear == 0)
                    {
                        car.gear = 1;
                    }
                    if (velocity.z > car.cartype.wheelradius * 2 * Mathf.PI * (0.9 * car.cartype.redLine) / (60 * car.cartype.gks[car.gear] * car.cartype.driveRatio))
                    {
                        if (car.gear < car.cartype.gks.Length - 1)
                        {
                            car.gear++;
                        }
                        else if (car.rpm == car.cartype.redLine)
                        {
                            car.throttle = 0;
                        }
                    }
                    else if (car.gear > 1 && velocity.z <= car.cartype.wheelradius * 2 * Mathf.PI * (0.9 * car.cartype.redLine) / (60 * car.cartype.gks[car.gear - 1] * car.cartype.driveRatio))
                    {
                        car.gear--;
                    }
                }
                else
                {
                    car.gear = 0;
                    if (car.rpm == car.cartype.redLine && velocity.z < 0)
                    {
                        car.throttle = 0;
                    }
                }
            }

            float Torque_w = car.cartype.GetTorqueAtRPM(car.rpm) * car.cartype.gks[car.gear] * car.cartype.driveRatio * (1.0f - car.cartype.drivetrainLoss);

            Torque_w *= car.throttle;

            float F_traction = Torque_w / car.cartype.wheelradius;

            if (car.brake > 0)
            {
                if (Mathf.Abs(velocity.z) > car.cartype.maxGrip * 9.8f * delta_t)
                {
                    F_traction = -Mathf.Sign(velocity.z) * car.cartype.Cbrake;
                }
                else
                {
                    F_traction = 0;
                }
            }
            if (car.rearSlip)
                F_traction *= 0.5f;

            Vector2 Frr = -car.cartype.Crr * car.cartype.mass * 9.8f * new Vector2(Mathf.Sign(velocity.z) * (0.008f + 3.7f * 10e-6f * velocity.z * velocity.z), Mathf.Sign(velocity.x) * (0.008f + 3.7f * 10e-6f * velocity.x * velocity.x));
            Vector2 Fdrag = -car.cartype.Cdrag * new Vector2(velocity.z * Mathf.Abs(velocity.z), velocity.x * Mathf.Abs(velocity.x));

            if (!car.isGrounded)
            {
                F_traction = 0;
                F_lat_r = Vector2.zero;
                F_lat_f = Vector2.zero;
                Frr = Vector2.zero;
            }

            Vector2 Fres = Frr + Fdrag;

            Vector3 Force = new Vector3(
                F_lat_r.x + F_lat_r.y + (F_lat_f.x + F_lat_f.y) * Mathf.Cos(car.steerangle) + Fres.y, 0.0f,
                F_traction - (F_lat_f.x + F_lat_f.y) * Mathf.Sin(car.steerangle) + Fres.x);

            Vector3 totalTorque = Vector3.zero;

            totalTorque.y = (F_lat_f.x + F_lat_f.y) * Mathf.Cos(car.steerangle) * car.cartype.b -
                (F_lat_r.x + F_lat_r.y) * car.cartype.c + (F_lat_f.x - F_lat_f.y) * Mathf.Sin(car.steerangle) * (car.cartype.track / 2);

            Vector3 acc = Force / car.cartype.mass;

            float Wt_acc = Utils.GetGlobalPosition(car, car.cartype.COM).y / car.cartype.wheelbase * car.cartype.mass * acc.z;
            Wt_acc = Mathf.Clamp(Wt_acc, -car.cartype.max_long_wt, car.cartype.max_long_wt);
            load[0] -= Wt_acc / 2;
            load[1] -= Wt_acc / 2;
            load[2] += Wt_acc / 2;
            load[3] += Wt_acc / 2;

            float Wt_lat_r = (acc.x / 9.8f) * (load[2] + load[3]) * Utils.GetGlobalPosition(car, car.cartype.COM).y / car.cartype.track;
            float Wt_lat_f = (acc.x / 9.8f) * (load[0] + load[1]) * Utils.GetGlobalPosition(car, car.cartype.COM).y / car.cartype.track;
            Wt_lat_r = Mathf.Clamp(Wt_lat_r, -car.cartype.max_lat_wt, car.cartype.max_lat_wt);
            Wt_lat_f = Mathf.Clamp(Wt_lat_f, -car.cartype.max_lat_wt, car.cartype.max_lat_wt);
            //float Wt_lat = Wt_lat_r + Wt_lat_f;
            load[0] += Wt_lat_f;
            load[1] -= Wt_lat_f;
            load[2] += Wt_lat_r;
            load[3] -= Wt_lat_r;

            Vector3 totalForce = Vector3.zero;
            List<Utils.ForceAtPosition> forces = new List<Utils.ForceAtPosition>();
            Vector3[] rayPoints = { Utils.GetGlobalPosition(car, new Vector3(-(car.cartype.track/2 - car.cartype.rayOffset_T_F), 0.0f, car.cartype.rayOffsetZ_F)),
                                Utils.GetGlobalPosition(car, new Vector3(car.cartype.track/2  - car.cartype.rayOffset_T_F, 0.0f, car.cartype.rayOffsetZ_F)),
                                Utils.GetGlobalPosition(car, new Vector3(-(car.cartype.track/2 -car.cartype.rayOffset_T_R), 0.0f, -car.cartype.rayOffsetZ_R)),
                                Utils.GetGlobalPosition(car, new Vector3(car.cartype.track/2  - car.cartype.rayOffset_T_R, 0.0f, -car.cartype.rayOffsetZ_R))
            }; // FL, FR, RL, RR
            Quaternion hit_RL = Quaternion.identity;
            Quaternion hit_RR = Quaternion.identity;
            int groundedWheels = 0;
            for (int i = 0; i < 4; i++)
            {
                forces.Add(new Utils.ForceAtPosition(-Vector3.up * load[i], rayPoints[i]));
                RaycastHit hit;
                float maxLength;
                if (i < 2)
                {
                    maxLength = car.cartype.restLength_F + car.cartype.springTravel + (1f - car.cartype.sphereCastRatio) * car.cartype.wheelradius;
                }
                else
                {
                    maxLength = car.cartype.restLength_R + car.cartype.springTravel + (1f - car.cartype.sphereCastRatio) * car.cartype.wheelradius;
                }
                Vector3 pointUp = car.rotation * Vector3.up;
                if (Physics.SphereCast(rayPoints[i], car.cartype.sphereCastRatio * car.cartype.wheelradius, -pointUp, out hit, maxLength))
                {
                    groundedWheels++;
                    car.cartype.wheels[i].position = rayPoints[i] + pointUp * (-hit.distance + (1f - car.cartype.sphereCastRatio) * car.cartype.wheelradius);
                    float currentSpringLength = hit.distance - (1f - car.cartype.sphereCastRatio) * car.cartype.wheelradius;
                    float springVelocity = Vector3.Dot(Utils.GetPointVelocity(car, rayPoints[i]), pointUp);
                    if (i < 2)
                    {
                        float springCompression = car.cartype.restLength_F - currentSpringLength;
                        float dampForce = car.cartype.DS_F * springVelocity;
                        float springForce = car.cartype.SS_F * springCompression;
                        if (Mathf.Abs(Vector3.Angle(hit.normal, pointUp)) <= car.cartype.suspension_activate_deg)
                            forces.Add(new Utils.ForceAtPosition(pointUp * (springForce - dampForce), rayPoints[i]));
                        else
                            forces.Add(new Utils.ForceAtPosition(car.cartype.weak_suspension_coef * hit.normal * (springForce - dampForce), rayPoints[i]));
                    }
                    else
                    {
                        Vector3 forwardOnSurface = Utils.ProjectOnPlane(car.rotation * Vector3.forward, hit.normal).normalized;
                        if (i == 2)
                            hit_RL = Utils.LookRotation(forwardOnSurface, hit.normal);
                        else
                            hit_RR = Utils.LookRotation(forwardOnSurface, hit.normal);
                        float springCompression = car.cartype.restLength_R - currentSpringLength;
                        float dampForce = car.cartype.DS_R * springVelocity;
                        float springForce = car.cartype.SS_R * springCompression;
                        if (Mathf.Abs(Vector3.Angle(hit.normal, pointUp)) <= car.cartype.suspension_activate_deg)
                            forces.Add(new Utils.ForceAtPosition(pointUp * (springForce - dampForce), rayPoints[i]));
                        else
                            forces.Add(new Utils.ForceAtPosition(car.cartype.weak_suspension_coef * hit.normal * (springForce - dampForce), rayPoints[i]));
                    }
                    //Debug.DrawLine(rayPoints[i], hit.point, Color.red);
                }
                else
                {
                    car.cartype.wheels[i].position = rayPoints[i] + pointUp * (-maxLength + (1f - car.cartype.sphereCastRatio) * car.cartype.wheelradius);
                    //Debug.DrawLine(rayPoints[i], rayPoints[i] + (car.cartype.sphereCastRatio * car.cartype.wheelradius + maxLength) * -pointUp, Color.green);
                }
            }
            car.isGrounded = groundedWheels == 4;
            Vector3 Force_w = Utils.TransformDirection(car.rotation, Force);
            if (car.isGrounded)
            {
                Force_w = Utils.TransformDirection(hit_RL, Force / 2) + Utils.TransformDirection(hit_RR, Force / 2);
            }

            float latVelocity = Utils.InverseTransformDirection(car.rotation, Force_w).x / car.cartype.mass * delta_t;

            forces.Add(new Utils.ForceAtPosition(Force_w, car.position_wc));
            foreach (var forceInfo in forces)
            {
                // Calculate the position of force application relative to the center of mass
                Vector3 forceOffset = forceInfo.forcePosition - Utils.GetGlobalPosition(car, car.cartype.COM);
                // Add the force to the total force
                totalForce += forceInfo.force;
                // Add the torque (r x F) to the total torque
                totalTorque += Vector3.Cross(forceOffset, forceInfo.force) * Mathf.Deg2Rad;
            }
            Vector3 acceleration_w = totalForce / car.cartype.mass;

            Vector3 angular_acc = new Vector3(totalTorque.x / car.cartype.inertia.x, totalTorque.y / car.cartype.inertia.y, totalTorque.z / car.cartype.inertia.z);

            car.rb.linearVelocity += acceleration_w * delta_t;
            car.rb.angularVelocity += angular_acc * delta_t;

            velocity = Utils.InverseTransformDirection(car.rotation, car.rb.linearVelocity);
            // Simple friction model
            if ((car.brake > 0 && Mathf.Abs(velocity.z) <= groundedWheels / 4 * car.cartype.maxGrip * 9.8f * delta_t) || Mathf.Abs(velocity.z) <= groundedWheels / 4 * car.cartype.Crr * 9.8f * 0.008f * delta_t)
            {
                velocity.z = 0;
                if (car.rb.angularVelocity.y > 0)
                {
                    if (car.rb.angularVelocity.y >= car.cartype.angular_friction)
                        car.rb.angularVelocity -= new Vector3(0, car.cartype.angular_friction, 0);
                    else
                    {
                        car.rb.angularVelocity = new Vector3(car.rb.angularVelocity.x, 0, car.rb.angularVelocity.z);
                    }
                }
                else if (car.rb.angularVelocity.y < 0)
                {
                    if (car.rb.angularVelocity.y <= -car.cartype.angular_friction)
                        car.rb.angularVelocity += new Vector3(0, car.cartype.angular_friction, 0);
                    else
                    {
                        car.rb.angularVelocity = new Vector3(car.rb.angularVelocity.x, 0, car.rb.angularVelocity.z);
                    }
                }
            }
            if (velocity.x > 0)
            {
                if (velocity.x - (car.expectedLatV + latVelocity) > 0)
                {
                    if (velocity.x - (car.expectedLatV + latVelocity) >= groundedWheels / 4 * car.cartype.maxGrip * 9.8f * delta_t)
                    {
                        velocity.x -= groundedWheels / 4 * car.cartype.maxGrip * 9.8f * delta_t;
                    }
                    else
                    {
                        velocity.x -= velocity.x - (car.expectedLatV + latVelocity);
                    }
                }
            }
            else if (velocity.x < 0)
            {
                if (velocity.x - (car.expectedLatV + latVelocity) < 0)
                {
                    if (velocity.x - (car.expectedLatV + latVelocity) <= groundedWheels / 4 * -car.cartype.maxGrip * 9.8f * delta_t)
                    {
                        velocity.x += groundedWheels / 4 * car.cartype.maxGrip * 9.8f * delta_t;
                    }
                    else
                    {
                        velocity.x -= velocity.x - (car.expectedLatV + latVelocity);
                    }
                }
            }
            car.rb.linearVelocity = Utils.TransformDirection(car.rotation, velocity);
            car.expectedLatV = velocity.x;
            car.wheelRot = velocity.z / car.cartype.wheelradius * Mathf.Rad2Deg;
            //car.position_wc += car.velocity_wc * delta_t;
            //Vector3 rotationDelta = car.angularVelocity * delta_t;
            //car.rotation = Quaternion.Euler(Mathf.Rad2Deg * rotationDelta) * car.rotation;
        }
    }
}
