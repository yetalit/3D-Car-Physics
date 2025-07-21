using UnityEngine;

namespace CPH
{
    public class Utils
    {
        public class ForceAtPosition
        {
            public Vector3 force;
            public Vector3 forcePosition;   // Position where the force is applied (in world space)

            public ForceAtPosition(Vector3 force, Vector3 forcePosition)
            {
                this.force = force;
                this.forcePosition = forcePosition;
            }
        }
        public static Vector3 GetPointVelocity(Car car, Vector3 point)
        {
            // Get the position vector relative to the object's center of mass
            Vector3 r = point - GetGlobalPosition(car, car.cartype.COM);
            // Calculate the point velocity
            return car.velocity_wc + Vector3.Cross(car.angularVelocity, r);
        }
        public static Vector3 InverseTransformDirection(Quaternion rotation, Vector3 worldDirection)
        {
            // Apply the inverse of the object's rotation to the world direction
            return Quaternion.Inverse(rotation) * worldDirection;
        }

        public static Vector3 TransformDirection(Quaternion rotation, Vector3 localDirection)
        {
            // Apply the object's rotation to the local direction
            return rotation * localDirection;
        }

        public static Vector3 ProjectOnPlane(Vector3 v, Vector3 normal)
        {
            // Make sure the normal is not zero to avoid division by zero
            if (normal == Vector3.zero)
            {
                return Vector3.zero;
            }
            // Normalize the normal to ensure we are projecting onto a unit normal
            normal.Normalize();
            // Compute the projection of v onto the normal (component of v along the normal)
            float dotProduct = Vector3.Dot(v, normal);
            // Subtract the projection from v to get the component of v on the plane
            Vector3 projection = v - dotProduct * normal;
            return projection;
        }
        public static Quaternion LookRotation(Vector3 forward, Vector3 up)
        {
            forward = forward.normalized;
            Vector3 right = Vector3.Cross(up, forward).normalized;
            Vector3 newUp = Vector3.Cross(forward, right);

            // Create rotation matrix
            Matrix4x4 rotMatrix = new Matrix4x4();
            rotMatrix.SetColumn(0, new Vector4(right.x, right.y, right.z, 0));
            rotMatrix.SetColumn(1, new Vector4(newUp.x, newUp.y, newUp.z, 0));
            rotMatrix.SetColumn(2, new Vector4(forward.x, forward.y, forward.z, 0));
            rotMatrix.SetColumn(3, new Vector4(0, 0, 0, 1));

            return QuaternionFromMatrix(rotMatrix);
        }
        private static Quaternion QuaternionFromMatrix(Matrix4x4 m)
        {
            Quaternion q = new Quaternion();
            q.w = Mathf.Sqrt(Mathf.Max(0, 1 + m.m00 + m.m11 + m.m22)) / 2;
            q.x = Mathf.Sqrt(Mathf.Max(0, 1 + m.m00 - m.m11 - m.m22)) / 2;
            q.y = Mathf.Sqrt(Mathf.Max(0, 1 - m.m00 + m.m11 - m.m22)) / 2;
            q.z = Mathf.Sqrt(Mathf.Max(0, 1 - m.m00 - m.m11 + m.m22)) / 2;

            q.x *= Mathf.Sign(q.x * (m.m21 - m.m12));
            q.y *= Mathf.Sign(q.y * (m.m02 - m.m20));
            q.z *= Mathf.Sign(q.z * (m.m10 - m.m01));
            return q;
        }

        public static Vector3 GetGlobalPosition(Car car, Vector3 LocalPosition)
        {
            // Add car's world position to get the global position
            return car.position_wc + TransformDirection(car.rotation, LocalPosition);
        }
    }
}
