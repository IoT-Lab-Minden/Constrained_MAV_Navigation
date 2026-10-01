using UnityEngine;

public abstract class SimDroneParameters : ScriptableObject
{
    public virtual void InitParameters() {}

    public abstract float GetTakeoffTime();
    public abstract float GetMaxHeight();
    public abstract float GetWeight();
}