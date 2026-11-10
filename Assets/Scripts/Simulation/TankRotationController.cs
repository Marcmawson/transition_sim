using UnityEngine;

namespace Seb.Fluid.Simulation
{
    /// <summary>
    /// Rotates the simulation bounds about the Simulation GameObject's pivot.
    /// Attach this to the same GameObject as FluidSim.
    /// </summary>
    public sealed class TankRotationController : MonoBehaviour
    {
        public enum RotationAxis
        {
            X,
            Y,
            Z
        }

        [Header("Tank Rotation")]
        [Tooltip("Axis in the tank's local coordinates.")]
        public RotationAxis axis = RotationAxis.X;

        [Tooltip("Target angle relative to the scene's starting orientation.")]
        [Range(-180f, 180f)]
        public float targetAngleDegrees;

        [Tooltip("Angle added or removed by each key press.")]
        [Min(0f)]
        public float degreesPerKeyPress = 5f;

        [Tooltip("Maximum rotation speed while moving toward the target angle.")]
        [Min(0f)]
        public float degreesPerSecond = 90f;

        [Tooltip("Use [ and ] to change the target angle, Backspace to return to zero.")]
        public bool keyboardInput = true;

        private Quaternion initialLocalRotation;

        private void Awake()
        {
            initialLocalRotation = transform.localRotation;
        }

        private void Update()
        {
            if (keyboardInput)
            {
                if (Input.GetKeyDown(KeyCode.LeftBracket))
                    targetAngleDegrees -= degreesPerKeyPress;

                if (Input.GetKeyDown(KeyCode.RightBracket))
                    targetAngleDegrees += degreesPerKeyPress;

                if (Input.GetKeyDown(KeyCode.Backspace))
                    targetAngleDegrees = 0f;

                targetAngleDegrees = Mathf.Clamp(targetAngleDegrees, -180f, 180f);
            }

            Vector3 localAxis = axis switch
            {
                RotationAxis.X => Vector3.right,
                RotationAxis.Y => Vector3.up,
                RotationAxis.Z => Vector3.forward,
                _ => Vector3.right
            };

            Quaternion targetRotation =
                initialLocalRotation * Quaternion.AngleAxis(targetAngleDegrees, localAxis);

            transform.localRotation = Quaternion.RotateTowards(
                transform.localRotation,
                targetRotation,
                degreesPerSecond * Time.deltaTime
            );
        }
    }
}
