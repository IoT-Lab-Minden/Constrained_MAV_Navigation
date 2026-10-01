using UnityEngine;

public abstract class KinematicDroneParameters : SimDroneParameters
{
    public abstract float GetLinearDragCoefficient();
    public abstract float GetAngularDragCoefficient();
    public abstract float GetLinearAcceleration();
    public abstract float GetAngularAcceleration();
    public abstract float GetMaxLinearVelocity();
    public abstract float GetMaxAngularVelocity();
}