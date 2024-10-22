using UnityEngine;

public class AI : MonoBehaviour
{
    private struct RayData
    {
        public Vector3 localDirection;
        public float length;
        public bool hitObstacle;
        public float hitDistance;
        public Vector3 direction;
    }

    [Header("State")]
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
    [SerializeField] private float _centralRayLength = 10f;
    [SerializeField] private float _peripheralRayLength = 15f;
    [SerializeField] private float _centralRayAngle = 5f;
    [SerializeField] private float _rayAngle = 30f;

    private RayData[] _centralRays;
    private RayData[] _peripheralRays;
    private bool centralRayHit = false;

    private void Start()
    {
        _centralRays = new RayData[5];
        _centralRays[0].localDirection = Vector3.forward;
        _centralRays[0].length = _centralRayLength;
        _centralRays[1].localDirection = Quaternion.Euler(0, -_centralRayAngle, 0) * Vector3.forward;
        _centralRays[1].length = _centralRayLength;
        _centralRays[2].localDirection = Quaternion.Euler(0, _centralRayAngle, 0) * Vector3.forward;
        _centralRays[2].length = _centralRayLength;
        _centralRays[3].localDirection = Quaternion.Euler(-_centralRayAngle, 0, 0) * Vector3.forward;
        _centralRays[3].length = _centralRayLength;
        _centralRays[4].localDirection = Quaternion.Euler(_centralRayAngle, 0, 0) * Vector3.forward;
        _centralRays[4].length = _centralRayLength;

        _peripheralRays = new RayData[12];
        _peripheralRays[0].localDirection = Quaternion.Euler(0, -_rayAngle, 0) * Vector3.forward;
        _peripheralRays[0].length = _peripheralRayLength;
        _peripheralRays[1].localDirection = Quaternion.Euler(0, _rayAngle, 0) * Vector3.forward;
        _peripheralRays[1].length = _peripheralRayLength;
        _peripheralRays[2].localDirection = Quaternion.Euler(-_rayAngle, 0, 0) * Vector3.forward;
        _peripheralRays[2].length = _peripheralRayLength;
        _peripheralRays[3].localDirection = Quaternion.Euler(_rayAngle, 0, 0) * Vector3.forward;
        _peripheralRays[3].length = _peripheralRayLength;

        float angle = 2 * _rayAngle;

        _peripheralRays[4].localDirection = Quaternion.Euler(0, angle, 0) * Vector3.forward;
        _peripheralRays[4].length = _peripheralRayLength;
        _peripheralRays[5].localDirection = Quaternion.Euler(-angle, angle, 0) * Vector3.forward;
        _peripheralRays[5].length = _peripheralRayLength;
        _peripheralRays[6].localDirection = Quaternion.Euler(-angle, 0, 0) * Vector3.forward;
        _peripheralRays[6].length = _peripheralRayLength;
        _peripheralRays[7].localDirection = Quaternion.Euler(-angle, -angle, 0) * Vector3.forward;
        _peripheralRays[7].length = _peripheralRayLength;
        _peripheralRays[8].localDirection = Quaternion.Euler(0, -angle, 0) * Vector3.forward;
        _peripheralRays[8].length = _peripheralRayLength;
        _peripheralRays[9].localDirection = Quaternion.Euler(angle, -angle, 0) * Vector3.forward;
        _peripheralRays[9].length = _peripheralRayLength;
        _peripheralRays[10].localDirection = Quaternion.Euler(angle, 0, 0) * Vector3.forward;
        _peripheralRays[10].length = _peripheralRayLength;
        _peripheralRays[11].localDirection = Quaternion.Euler(angle, angle, 0) * Vector3.forward;
        _peripheralRays[11].length = _peripheralRayLength;
    }

    private void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;
        CheckDistance();

        if (!_enable)
            return;

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

    private void CheckDistance()
    {
        if (_target != null)
        {
            float distanceToTarget = Vector3.Distance(transform.position, _target.position);
            if (distanceToTarget <= _distanceToStop)
            {
                _enable = false;
            }
        }
    }

    private void CastRays()
    {
        Vector3 rayOrigin = transform.position;
        centralRayHit = false;

        for (int i = 0; i < _centralRays.Length; i++)
        {
            Vector3 worldDirection = transform.TransformDirection(_centralRays[i].localDirection);
            _centralRays[i].direction = worldDirection;

            RaycastHit hit;
            bool rayHit = Physics.Raycast(rayOrigin, worldDirection, out hit, _centralRays[i].length);

            _centralRays[i].hitObstacle = rayHit;
            _centralRays[i].hitDistance = rayHit ? hit.distance : _centralRays[i].length;

            Color rayColor = rayHit ? Color.red : Color.green;
            Debug.DrawRay(rayOrigin, worldDirection * _centralRays[i].length, rayColor);

            if (rayHit)
            {
                centralRayHit = true;
            }
        }

        for (int i = 0; i < _peripheralRays.Length; i++)
        {
            Vector3 worldDirection = transform.TransformDirection(_peripheralRays[i].localDirection);
            _peripheralRays[i].direction = worldDirection;

            RaycastHit hit;
            bool rayHit = Physics.Raycast(rayOrigin, worldDirection, out hit, _peripheralRays[i].length);

            _peripheralRays[i].hitObstacle = rayHit;
            _peripheralRays[i].hitDistance = rayHit ? hit.distance : _peripheralRays[i].length;

            Color rayColor = rayHit ? Color.red : Color.green;
            Debug.DrawRay(rayOrigin, worldDirection * _peripheralRays[i].length, rayColor);
        }
    }

    private void MoveForward(float dt, float multiple = 1)
    {
        Vector3 directionToTarget = _target.position - transform.position;
        float distanceToTarget = directionToTarget.magnitude;

        if (distanceToTarget > _distanceToStop)
        {
            float forceMagnitude = _pid.Calculate(distanceToTarget, dt) * _pid.Force * multiple;
            forceMagnitude = Mathf.Min(forceMagnitude, _maxForce);

            Vector3 forwardForce = transform.forward * forceMagnitude;
            _rigidbody.AddForce(forwardForce);
        }
    }

    private void ApplyRotationTorque()
    {
        Vector3 directionToTarget = (_target.position - transform.position).normalized;
        ApplyTorqueTowardsDirection(directionToTarget);
    }

    private void ApplyTorqueTowardsDirection(Vector3 targetDirection, float torqueMultiplier = 1f)
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

    private void AvoidObstacle()
    {
        for (int i = 0; i < _peripheralRays.Length; i++)
        {
            if (!_peripheralRays[i].hitObstacle)
            {
                ApplyTorqueTowardsDirection(_peripheralRays[i].direction, _avoidanceTorqueMultiplier);
                return;
            }
        }

        float maxDistance = 0f;
        int selectedIndex = -1;

        for (int i = 0; i < _peripheralRays.Length; i++)
        {
            if (_peripheralRays[i].hitDistance > maxDistance)
            {
                maxDistance = _peripheralRays[i].hitDistance;
                selectedIndex = i;
            }
        }

        if (selectedIndex != -1)
        {
            ApplyTorqueTowardsDirection(_peripheralRays[selectedIndex].direction, _avoidanceTorqueMultiplier);
        }
    }
}
