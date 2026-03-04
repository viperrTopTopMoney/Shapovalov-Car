using UnityEngine;

public class KartEngine : MonoBehaviour
{
    [Header("Import parametrs")]
    [SerializeField] private bool _import = false;
    [SerializeField] private KartConfig _kartConfig;
    [Header("RPM settings")]
    [SerializeField] private float _idleRpm = 1000f;
    [SerializeField] private float _maxRpm = 8000f;
    [SerializeField] private float _revLimiterRpm = 7500f;

    [Header("Torque curve")]
    [SerializeField] private AnimationCurve _torqueCurve;

    [Header("Inertia & response")]
    [SerializeField] private float _flywheelInertia = 0.2f;
    [SerializeField] private float _throttleResponse = 5f;

    [Header("Losses & load")]
    [SerializeField] private float _engineFrictionCoeff = 0.02f;
    [SerializeField] private float _loadTorqueCoeff = 5f;

    public float CurrentRpm { get; private set; }
    public float CurrentTorque { get; private set; }
    public float SmoothedThrottle { get; private set; }
    public float RevLimiterFactor { get; private set; } = 1f;

    private float _invInertiaFactor;

    private void Awake()
    {
        if (_import)
        {
            Initialize();
        }
        CurrentRpm = _idleRpm;
        _invInertiaFactor = 60f / (2f * Mathf.PI * Mathf.Max(_flywheelInertia, 0.0001f));
    }

    private void Initialize()
    {
        if (_kartConfig != null)
        {
            _torqueCurve = _kartConfig.engineTorqueCurve;
            if (_kartConfig.engineInertia > 0)
                _invInertiaFactor = 60f / (2f * Mathf.PI * Mathf.Max(_kartConfig.engineInertia, 0.0001f));
            _maxRpm = _kartConfig.maxRpm;
        }
    }
    public float Simulate(float throttleInput, float forwardSpeed, float deltaTime)
    {
        float targetThrottle = Mathf.Clamp01(throttleInput);
        SmoothedThrottle = Mathf.MoveTowards(SmoothedThrottle, targetThrottle, _throttleResponse * deltaTime);

        UpdateRevLimiterFactor();

        float maxTorqueAtRpm = _torqueCurve.Evaluate(CurrentRpm);

        float effectiveThrottle = SmoothedThrottle * RevLimiterFactor;
        float driveTorque = maxTorqueAtRpm * effectiveThrottle;

        float frictionTorque = _engineFrictionCoeff * CurrentRpm;
        float loadTorque = _loadTorqueCoeff * Mathf.Abs(forwardSpeed);

        float netTorque = driveTorque - frictionTorque - loadTorque;

        float rpmDot = netTorque * _invInertiaFactor;
        CurrentRpm += rpmDot * deltaTime;

        if (CurrentRpm < _idleRpm) CurrentRpm = _idleRpm;
        if (CurrentRpm > _maxRpm) CurrentRpm = _maxRpm;

        CurrentTorque = driveTorque;
        return CurrentTorque;
    }

    private void UpdateRevLimiterFactor()
    {
        if (CurrentRpm <= _revLimiterRpm)
        {
            RevLimiterFactor = 1f;
            return;
        }

        if (CurrentRpm >= _maxRpm)
        {
            RevLimiterFactor = 0f;
            return;
        }

        float t = (CurrentRpm - _revLimiterRpm) / (_maxRpm - _revLimiterRpm);
        RevLimiterFactor = 1f - t;
    }
}