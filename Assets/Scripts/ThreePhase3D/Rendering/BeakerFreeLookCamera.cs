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
        private float homeSize, homeFieldOfView, yaw, pitch;
        private bool looking;
        public bool IsUndersideView { get; private set; }
        private Vector3 freePosition;
        private Quaternion freeRotation;
        private float freeSize;
        private CursorLockMode priorLock;
        private bool priorVisibility;

        private void Awake()
        {
            view = GetComponent<Camera>();
            InvertMouseY = PlayerPrefs.GetInt(InvertMouseKey, 0) != 0;
            homePosition = transform.position;
            homeRotation = transform.rotation;
            homeSize = view.orthographicSize;
            homeFieldOfView = view.fieldOfView;
            SetLookAngles();
        }

        private void Update()
        {
            if (!Application.isFocused || IsUndersideView) return;
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
                view.fieldOfView = homeFieldOfView;
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
            {
                float zoom = Mathf.Pow(.85f, scroll);
                if (view.orthographic)
                    view.orthographicSize = Mathf.Clamp(view.orthographicSize * zoom, .02f, .65f);
                else
                    view.fieldOfView = Mathf.Clamp(view.fieldOfView * zoom, 15f, 85f);
            }

            var movement = transform.right * Axis(KeyCode.D, KeyCode.A) +
                transform.forward * Axis(KeyCode.W, KeyCode.S) + Vector3.up * Axis(KeyCode.E, KeyCode.Q);
            float speed = view.orthographicSize * 1.5f;
            if (Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift)) speed *= 3f;
            transform.position += Vector3.ClampMagnitude(movement, 1f) * (speed * Mathf.Min(Time.unscaledDeltaTime, .1f));
        }

        public void FrameRelease(Vector3 target, float height)
        {
            ReleaseMouse();
            IsUndersideView = false;
            var rotation = homeRotation;
            float distance = Mathf.Max(.18f, height / (2f * Mathf.Tan(homeFieldOfView * Mathf.Deg2Rad / 2f)) * 1.35f);
            transform.SetPositionAndRotation(target - rotation * Vector3.forward * distance, rotation);
            view.fieldOfView = homeFieldOfView;
            view.orthographicSize = Mathf.Max(.08f, height * .68f);
            SetLookAngles();
        }

        public void ToggleUndersideView(float baseHeight, float outerRadius)
        {
            if (IsUndersideView)
            {
                transform.SetPositionAndRotation(freePosition, freeRotation);
                view.orthographicSize = freeSize;
                IsUndersideView = false;
                SetLookAngles();
                return;
            }

            ReleaseMouse();
            freePosition = transform.position;
            freeRotation = transform.rotation;
            freeSize = view.orthographicSize;
            Vector3 target = Vector3.up * baseHeight;
            Vector3 offset = new Vector3(1.1f, -1.6f, -.9f) * outerRadius;
            transform.SetPositionAndRotation(target + offset, Quaternion.LookRotation(-offset, Vector3.up));
            view.orthographicSize = 1.25f * outerRadius;
            IsUndersideView = true;
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
