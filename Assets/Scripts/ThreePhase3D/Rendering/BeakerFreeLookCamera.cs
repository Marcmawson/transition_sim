using UnityEngine;

namespace PhysicsSim.ThreePhase3D.Rendering
{
    [RequireComponent(typeof(Camera))]
    public sealed class BeakerFreeLookCamera : MonoBehaviour
    {
        private const string InvertMouseKey = "PhysicsSim.S07.InvertMouseY";
        private Camera view;
        public bool InvertMouseY { get; private set; }
        private Vector3 homePosition;
        private Quaternion homeRotation;
        private float homeSize, yaw, pitch;
        private bool looking;
        private CursorLockMode priorLock;
        private bool priorVisibility;

        private void Awake()
        {
            view = GetComponent<Camera>();
            InvertMouseY = PlayerPrefs.GetInt(InvertMouseKey, 0) != 0;
            homePosition = transform.position;
            homeRotation = transform.rotation;
            homeSize = view.orthographicSize;
            SetLookAngles();
        }

        private void Update()
        {
            if (!Application.isFocused) return;
            if (Input.GetKeyDown(KeyCode.I))
            {
                InvertMouseY = !InvertMouseY;
                PlayerPrefs.SetInt(InvertMouseKey, InvertMouseY ? 1 : 0);
                PlayerPrefs.Save();
            }
            if (Input.GetKeyDown(KeyCode.R))
            {
                transform.SetPositionAndRotation(homePosition, homeRotation);
                view.orthographicSize = homeSize;
                SetLookAngles();
            }

            if (looking && (!Input.GetMouseButton(1) || Input.GetKeyDown(KeyCode.Escape) ||
                Cursor.lockState != CursorLockMode.Locked))
                ReleaseMouse();
            if (Input.GetMouseButtonDown(1) && !looking)
            {
                priorLock = Cursor.lockState;
                priorVisibility = Cursor.visible;
                Cursor.lockState = CursorLockMode.Locked;
                Cursor.visible = false;
                looking = true;
            }
            else if (looking)
            {
                yaw += Input.GetAxisRaw("Mouse X") * 2f;
                pitch = Mathf.Clamp(pitch + Input.GetAxisRaw("Mouse Y") * (InvertMouseY ? 2f : -2f), -85f, 85f);
                transform.rotation = Quaternion.Euler(pitch, yaw, 0);
            }

            float scroll = Input.mouseScrollDelta.y;
            if (scroll != 0)
                view.orthographicSize = Mathf.Clamp(view.orthographicSize * Mathf.Pow(.85f, scroll), .02f, .65f);

            var movement = transform.right * Axis(KeyCode.D, KeyCode.A) +
                transform.forward * Axis(KeyCode.W, KeyCode.S) + Vector3.up * Axis(KeyCode.E, KeyCode.Q);
            float speed = view.orthographicSize * 1.5f;
            if (Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift)) speed *= 3f;
            transform.position += Vector3.ClampMagnitude(movement, 1f) * (speed * Mathf.Min(Time.unscaledDeltaTime, .1f));
        }

        private static int Axis(KeyCode positive, KeyCode negative) =>
            (Input.GetKey(positive) ? 1 : 0) - (Input.GetKey(negative) ? 1 : 0);

        private void SetLookAngles()
        {
            var angles = transform.eulerAngles;
            yaw = angles.y;
            pitch = Mathf.DeltaAngle(0, angles.x);
        }

        private void ReleaseMouse()
        {
            if (!looking) return;
            looking = false;
            Cursor.lockState = priorLock;
            Cursor.visible = priorVisibility;
        }

        private void OnApplicationFocus(bool focused)
        {
            if (!focused) ReleaseMouse();
        }

        private void OnDisable() => ReleaseMouse();
    }
}
