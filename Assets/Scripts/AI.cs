using UnityEngine;

public class AI : MonoBehaviour
{
    [Header ("State")]
    [SerializeField] private bool _enable = false;

    [Header("Components")]
    [SerializeField] private Rigidbody _rigidbody;
    [SerializeField] private Simple_PID _pid;

    [SerializeField] private Transform _target;

    [Header("Parameters Movement")]
    [SerializeField] private float _distanceToStop = 0.3f;
    [SerializeField] private float _maxForce = 10f;
    [SerializeField] private float _avoidanceForceMultiplier = 0.3f;
    [SerializeField] private float _maxRotationTorque = 3f;
    [SerializeField] private float _avoidanceTorqueMultiplier = 3f;

    [Header("Ray Parameters")]
    [SerializeField] private float _rayLength = 15f;
    [SerializeField] private float _rayAngle = 30f;
    [SerializeField] private float _verticalAngleMultiply = 10f;

    private struct RayData
    {
        public Vector3 direction;
        public bool hitObstacle;
        public float hitDistance;
        public string name;
    }

    private RayData[] rays;
    private Quaternion[] rayRotations;
    private string[] rayNames = { "Center", "Left", "Right", "Up", "Down" };

    private bool centralRayHit = false;

    void Start()
    {
        rays = new RayData[5];
        rayRotations = new Quaternion[5];

        rayRotations[0] = Quaternion.identity;

        rayRotations[1] = Quaternion.Euler(0, -_rayAngle, 0); // ¬лево
        rayRotations[2] = Quaternion.Euler(0, _rayAngle, 0);  // ¬право
        rayRotations[3] = Quaternion.Euler(-_rayAngle * _verticalAngleMultiply, 0, 0); // ¬верх
        rayRotations[4] = Quaternion.Euler(_rayAngle * _verticalAngleMultiply, 0, 0);  // ¬низ
    }

    void FixedUpdate()
    {
        if (!_enable)
            return;

        float dt = Time.fixedDeltaTime;

        if (_target != null)
        {
            float distanceToTarget = Vector3.Distance(transform.position, _target.position);
            if (distanceToTarget <= _distanceToStop)
            {
                _enable = false;
                return;
            }
        }

        CastRays();

        if (_target != null)
        {
            if (!centralRayHit)
            {
                MoveForward(dt);
                ApplyRotationTorque();
            }
            else
            {
                MoveForward(dt, _avoidanceForceMultiplier);
                AvoidObstacle();
            }
        }
    }

    void CastRays()
    {
        Vector3 rayOrigin = transform.position;
        Vector3 forwardDirection = transform.forward;

        centralRayHit = false;

        for (int i = 0; i < rays.Length; i++)
        {
            rays[i].direction = rayRotations[i] * forwardDirection;
            rays[i].name = rayNames[i];

            RaycastHit hit;
            bool rayHit = Physics.Raycast(rayOrigin, rays[i].direction, out hit, _rayLength);

            rays[i].hitObstacle = rayHit;
            if (rayHit)
            {
                rays[i].hitDistance = hit.distance;
            }
            else
            {
                rays[i].hitDistance = _rayLength;
            }

            Color rayColor = rayHit ? Color.red : Color.green;
            Debug.DrawRay(rayOrigin, rays[i].direction * _rayLength, rayColor);

            if (i == 0)
            {
                centralRayHit = rayHit;
            }
        }
    }

    public void MoveForward(float dt, float multiple = 1)
    {
        Vector3 directionToTarget = _target.position - transform.position;
        float distanceToTarget = directionToTarget.magnitude;

        if (distanceToTarget > _distanceToStop)
        {
            float forceMagnitude = _pid.Calculate(distanceToTarget, dt) * _pid.Force;

            forceMagnitude = Mathf.Min(forceMagnitude, _maxForce);

            Vector3 forwardForce = transform.forward * forceMagnitude;
            _rigidbody.AddForce(forwardForce);
        }
    }

    public void ApplyRotationTorque()
    {
        Vector3 directionToTarget = (_target.position - transform.position).normalized;
        ApplyTorqueTowardsDirection(directionToTarget);
    }

    void ApplyTorqueTowardsDirection(Vector3 targetDirection, float torqueMultiplier = 1f)
    {
        Vector3 forward = transform.forward;
        Vector3 rotationAxis = Vector3.Cross(forward, targetDirection);

        float angle = Vector3.Angle(forward, targetDirection);

        if (rotationAxis == Vector3.zero || angle == 0f)
        {
            return;
        }

        rotationAxis = rotationAxis.normalized;

        float torqueMagnitude = (angle / 180f) * _maxRotationTorque * torqueMultiplier;

        _rigidbody.AddTorque(rotationAxis * torqueMagnitude, ForceMode.Force);
    }

    public void AvoidObstacle()
    {
        for (int i = 1; i < rays.Length; i++)
        {
            if (!rays[i].hitObstacle)
            {
                ApplyTorqueTowardsDirection(rays[i].direction, _avoidanceTorqueMultiplier);
                return;
            }
        }

        float minDistance = float.MaxValue;
        int selectedIndex = -1;

        for (int i = 1; i < rays.Length; i++)
        {
            if (rays[i].hitDistance < minDistance)
            {
                minDistance = rays[i].hitDistance;
                selectedIndex = i;
            }
        }

        ApplyTorqueTowardsDirection(rays[selectedIndex].direction, _avoidanceTorqueMultiplier);
    }
}
