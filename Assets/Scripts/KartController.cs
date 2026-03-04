using UnityEngine;
using UnityEngine.InputSystem;

[RequireComponent(typeof(Rigidbody))]
public class KartController : MonoBehaviour
{
    [Header("Import parametrs")]
    [SerializeField] private bool _import = false;
    [SerializeField] private KartConfig _kartConfig;

    [Header("Wheel attachment points")]
    [SerializeField] private Transform _frontLeftWheel;
    [SerializeField] private Transform _frontRightWheel;
    [SerializeField] private Transform _rearLeftWheel;
    [SerializeField] private Transform _rearRightWheel;

    [Header("Input (New Input System)")]
    [SerializeField] private InputActionAsset _playerInput;

    [Header("Weight distribution")]
    [SerializeField, Range(0, 1)] private float _frontAxisShare = 0.5f;

    [Header("Engine & drivetrain")]
    [SerializeField] private KartEngine _engine;
    [SerializeField] private float _gearRatio = 8f;
    [SerializeField] private float _drivetrainEfficiency = 0.9f;

    private InputAction _moveAction;
    private float _throttleInput;
    private float _steepInput;
    private bool _handbrakePressed;

    private float _frontLeftNormalForce, _frontRightNormalForce, _rearLeftNormalForce, _rearRightNormalForce;
    private Rigidbody _rigidbody;
    private Vector3 g = Physics.gravity;

    [SerializeField] private float engineTorque = 400f;
    [SerializeField] private float wheelRadius = 0.3f;
    [SerializeField] private float maxSpeed = 20;

    [Header("Steering")]
    [SerializeField] private float maxSteeringAngle;

    private Quaternion frontLeftInitialRot;
    private Quaternion frontRightInitialRot;

    [Header("Tyre friction")]
    [SerializeField] private float frictionCoefficient = 1f;
    [SerializeField] private float lateralStiffnes = 80f;
    [SerializeField] private float rollingResistance = 30f;

    [Header("Handbrake")]
    [SerializeField] private float handbrakeRollingMultiplier = 8f;

    private float speedAlongForward = 0f;
    private float Fx = 0f;
    private float Fy = 0f;

    private float _telemetryTotalFx;
    private float _telemetryTotalFy;

    private void Awake()
    {
        _playerInput.Enable();
        _rigidbody = GetComponent<Rigidbody>();

        var map = _playerInput.FindActionMap("Kart");
        _moveAction = map.FindAction("Move");

        if (_import) Initialize();

        frontLeftInitialRot = _frontLeftWheel.localRotation;
        frontRightInitialRot = _frontRightWheel.localRotation;

        ComputeStaticWheelLoad();

        if (_engine == null) _engine = GetComponent<KartEngine>();
    }

    private void Initialize()
    {
        if (_kartConfig != null)
        {
            _rigidbody.mass = _kartConfig.mass;
            frictionCoefficient = _kartConfig.frictionCoefficient;
            rollingResistance = _kartConfig.rollingResistance;
            maxSteeringAngle = _kartConfig.maxSteerAngle;
            _gearRatio = _kartConfig.gearRatio;
            wheelRadius = _kartConfig.wheelRadius;
            lateralStiffnes = _kartConfig.lateralStiffness;
        }
    }

    private void OnDisable()
    {
        _playerInput.Disable();
    }

    private void Update()
    {
        ReadInput();
        RotateFrontWheels();
    }

    private void ReadInput()
    {
        Vector2 move = _moveAction.ReadValue<Vector2>();
        _steepInput = Mathf.Clamp(move.x, -1, 1);
        _throttleInput = Mathf.Clamp(move.y, -1, 1);
        _handbrakePressed = Input.GetKey(KeyCode.Space);
    }

    void RotateFrontWheels()
    {
        float steerAngle = maxSteeringAngle * _steepInput;
        Quaternion steerRot = Quaternion.Euler(0, steerAngle, 0);
        _frontLeftWheel.localRotation = frontLeftInitialRot * steerRot;
        _frontRightWheel.localRotation = frontRightInitialRot * steerRot;
    }

    void ComputeStaticWheelLoad()
    {
        float mass = _rigidbody.mass;
        float totalWeight = mass * Mathf.Abs(g.y);
        float frontWeight = totalWeight * _frontAxisShare;
        float rearWeight = totalWeight - frontWeight;

        _frontRightNormalForce = _frontLeftNormalForce = frontWeight * 0.5f;
        _rearRightNormalForce = _rearLeftNormalForce = rearWeight * 0.5f;
    }

    private void ApplyEngineForces()
    {
        Vector3 forward = transform.forward;
        float speedAlongForward = Vector3.Dot(_rigidbody.linearVelocity, forward);
        if (_throttleInput > 0 && speedAlongForward > maxSpeed) return;
        if (_throttleInput < 0 && speedAlongForward < -5f) return;

        float driveTorque = engineTorque * _throttleInput;

        float safeRadius = Mathf.Max(wheelRadius, 0.01f);
        float driveForcePerWheel = driveTorque / safeRadius / 2;

        Vector3 forceRear = forward * driveForcePerWheel;

        _rigidbody.AddForceAtPosition(forceRear, _rearLeftWheel.position, ForceMode.Force);
        _rigidbody.AddForceAtPosition(forceRear, _rearRightWheel.position, ForceMode.Force);
    }

    private void FixedUpdate()
    {
        _telemetryTotalFx = 0f;
        _telemetryTotalFy = 0f;

        ApplyEngineForces();

        ApplyWheelForce(_frontLeftWheel, _frontLeftNormalForce, isSteer: true, isDrive: false);
        ApplyWheelForce(_frontRightWheel, _frontRightNormalForce, isSteer: true, isDrive: false);
        ApplyWheelForce(_rearLeftWheel, _rearLeftNormalForce, isSteer: false, isDrive: true);
        ApplyWheelForce(_rearRightWheel, _rearRightNormalForce, isSteer: false, isDrive: true);

        Fx = _telemetryTotalFx;
        Fy = _telemetryTotalFy;
    }

    void ApplyWheelForce(Transform wheel, float normalForce, bool isSteer, bool isDrive)
    {
        Vector3 wheelPos = wheel.position;
        Vector3 wheelForward = wheel.forward;
        Vector3 wheelRight = wheel.right;
        Vector3 velocity = _rigidbody.GetPointVelocity(wheelPos);
        float vlong = Vector3.Dot(velocity, wheelForward);
        float vlat = Vector3.Dot(velocity, wheelRight);

        float currentFx = 0f;
        float currentFy = 0f;

        bool isRear = (wheel == _rearLeftWheel || wheel == _rearRightWheel);
        float currentLateralStiffness = lateralStiffnes;
        float currentRollingResistance = rollingResistance;

        if (isRear && _handbrakePressed)
        {
            currentLateralStiffness = 0f;
            currentRollingResistance *= handbrakeRollingMultiplier;

            float additionalBrakeForce = 1200f;
            if (Mathf.Abs(vlong) > 0.5f)
            {
                float brakeDir = vlong > 0 ? -1f : 1f;
                currentFx += brakeDir * additionalBrakeForce;
            }
        }

        if (isDrive)
        {
            speedAlongForward = Vector3.Dot(_rigidbody.linearVelocity, transform.forward);
            float engineTorqueOut = _engine.Simulate(_throttleInput, speedAlongForward, Time.fixedDeltaTime);
            float totalWheelTorque = engineTorqueOut * _gearRatio * _drivetrainEfficiency;
            float wheelTorque = totalWheelTorque * 0.5f;

            float safeRadius = Mathf.Max(wheelRadius, 0.01f);
            currentFx += wheelTorque / safeRadius;
        }

        if (isSteer)
        {
            float rooling = -currentRollingResistance * vlong;
            currentFx += rooling;
        }

        float fyRaw = -currentLateralStiffness * vlat;
        currentFy += fyRaw;

        float frictionlimit = frictionCoefficient * normalForce;
        float forceLenght = Mathf.Sqrt(currentFx * currentFx + currentFy * currentFy);

        if (forceLenght > frictionlimit)
        {
            float scale = frictionlimit / forceLenght;
            currentFy += scale;  
            currentFx += scale;  
        }

        Vector3 force = wheelForward * currentFx + wheelRight * currentFy;
        _rigidbody.AddForceAtPosition(force, wheel.position, ForceMode.Force);

        if (isRear) _telemetryTotalFx += currentFx;
        if (isSteer) _telemetryTotalFy += currentFy;
    }

    private void OnGUI()
    {
        GUIStyle labelStyle = new GUIStyle(GUI.skin.label) { fontSize = 14, fontStyle = FontStyle.Bold, alignment = TextAnchor.MiddleLeft };
        labelStyle.normal.textColor = Color.white;

        GUIStyle headerStyle = new GUIStyle(GUI.skin.label) { fontSize = 18, fontStyle = FontStyle.Bold, alignment = TextAnchor.MiddleCenter };
        headerStyle.normal.textColor = Color.red;

        float boxW = 300, boxH = 200;
        GUILayout.BeginArea(new Rect(20, 20, boxW, boxH), GUI.skin.box);

        GUILayout.Label("KART DASHBOARD", headerStyle);
        GUILayout.Space(10);

        float speedKmh = speedAlongForward * 3.6f;
        DrawStatBar("Speed", speedKmh, 100f, "km/h", Color.blue);

        float rpmPct = _engine.CurrentRpm / 8000f;
        Color rpmColor = rpmPct > 0.9f ? Color.red : Color.green;
        DrawStatBar("RPM", _engine.CurrentRpm, 8000f, "", rpmColor);

        GUILayout.BeginVertical("box");
        GUILayout.Label($"Torque: {_engine.CurrentTorque:F1} Nm", labelStyle);
        GUILayout.Label($"R-Force (Fx): {Fx:F0} N", labelStyle);
        GUILayout.Label($"F-Steer (Fy): {Fy:F0} N", labelStyle);
        GUILayout.EndVertical();

        if (_handbrakePressed)
        {
            GUI.color = Color.red;
            GUILayout.Box("HANDBRAKE", GUILayout.ExpandWidth(true));
            GUI.color = Color.white;
        }

        GUILayout.EndArea();
    }

    private void DrawStatBar(string label, float value, float max, string unit, Color barColor)
    {
        GUILayout.BeginHorizontal();
        GUILayout.Label($"{label}: {value:F0} {unit}", GUILayout.Width(120));

        float pct = Mathf.Clamp01(Mathf.Abs(value) / max);
        Rect r = GUILayoutUtility.GetRect(100, 20);

        GUI.DrawTexture(r, Texture2D.grayTexture);
        Rect fill = new Rect(r.x, r.y, r.width * pct, r.height);
        Color old = GUI.color;
        GUI.color = barColor;
        GUI.DrawTexture(fill, Texture2D.whiteTexture);
        GUI.color = old;

        GUILayout.EndHorizontal();
        GUILayout.Space(4);
    }
}