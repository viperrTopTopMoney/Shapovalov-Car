using UnityEngine;

[CreateAssetMenu(fileName = "KartConfig", menuName = "Scriptable Objects/KartConfig")]
public class KartConfig : ScriptableObject
{
    public float mass;
    public float frictionCoefficient;
    public float lateralStiffness;
    public float rollingResistance;
    public float maxSteerAngle;
    public AnimationCurve engineTorqueCurve;
    public float engineInertia;
    public float maxRpm;
    public float gearRatio;
    public float wheelRadius;
}
