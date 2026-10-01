using UnityEngine;

[CreateAssetMenu(menuName = "Drone/Parameters/Kinematic/Static")]
public class StaticKinematicDroneParameters : KinematicDroneParameters
{
    [Header("Height Parameters")]
    public float takeoffTime = 0.85f;
    public float maxHeight = 5.0f;

    [Header("Weight Parameters")]
    public float weight = 0.08f;

    [Header("Drag Coefficients")]
    public float linearDragCoefficient = 10.0f;
    public float angularDragCoefficient = 5.0f;

    [Header("Acceleration Parameters")]
    public float linearAcceleration = 15.0f;
    public float angularAcceleration = 20.0f;

    [Header("Max Velocity Parameters")]
    public float maxLinearVelocity = 1.5f;
    public float maxAngularVelocity = 20.0f;

    public override float GetAngularAcceleration()
    {
        return angularAcceleration;
    }

    public override float GetAngularDragCoefficient()
    {
        return angularDragCoefficient;
    }

    public override float GetLinearAcceleration()
    {
        return linearAcceleration;
    }

    public override float GetLinearDragCoefficient()
    {
        return linearDragCoefficient;
    }

    public override float GetMaxAngularVelocity()
    {
        return maxAngularVelocity;
    }

    public override float GetMaxHeight()
    {
        return maxHeight;
    }

    public override float GetMaxLinearVelocity()
    {
        return maxLinearVelocity;
    }

    public override float GetTakeoffTime()
    {
        return takeoffTime;
    }

    public override float GetWeight()
    {
        return weight;
    }
}
