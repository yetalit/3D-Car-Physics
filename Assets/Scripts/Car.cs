using UnityEngine;

namespace CPH
{
    [RequireComponent(typeof(CarType))]
    public class Car : MonoBehaviour
    {
        [HideInInspector]
        public CarType cartype;
        [HideInInspector]
        public Rigidbody rb;
        [HideInInspector]
        public Vector3 position_wc;   // World coordinates
        [HideInInspector]
        public Vector3 velocity_wc;   // World velocity
        [HideInInspector]
        public Quaternion rotation;
        [HideInInspector]
        public Vector3 angularVelocity;
        public int gear;
        public int rpm;
        [HideInInspector]
        public float wheelRot;
        public float steerangle;
        public float throttle;
        public float brake;
        public bool isGrounded;
        public bool rearSlip;
        public bool frontSlip;
        [HideInInspector]
        public float expectedLatV;

        public void setUp(Vector3 position, Quaternion rotation)
        {
            cartype = GetComponent<CarType>();
            cartype.setUp();

            rb = gameObject.AddComponent<Rigidbody>();
            rb.mass = cartype.mass / 10; // Dividing to reduce the effect of collisions for better handling
            rb.angularDamping = 0;
            rb.automaticCenterOfMass = false;
            rb.automaticInertiaTensor = false;
            rb.inertiaTensorRotation = Quaternion.identity;
            rb.useGravity = false;

            position_wc = position;
            velocity_wc = Vector3.zero;
            this.rotation = rotation;
            angularVelocity = Vector3.zero;
            gear = 1;
            rpm = 6;
            wheelRot = 0;
            steerangle = 0;
            throttle = 0;
            brake = 0;
            isGrounded = false;
            rearSlip = false;
            frontSlip = false;
            expectedLatV = 0;
        }
    }
}
