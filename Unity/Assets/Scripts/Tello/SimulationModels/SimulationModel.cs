using UnityEngine;

public abstract class SimulationModel : ScriptableObject
{
    public abstract SimDroneParameters GetDroneParameters();

    public abstract void ResetSimulationModel(Transform drone);

    public abstract void UpdatePosition(ControllerState controllerState, Transform drone);

    public abstract void UpdateOrientation(ControllerState controllerState, Transform drone);

    public abstract Vector3 GetLinearVelocity();
    
    public abstract float GetAngularVelocty();

    public virtual void InitializePredictionState(Transform drone, Vector3 linearVelocity, float angularVelocity)
    {
        ResetSimulationModel(drone);
    }

    public static T Clone<T>(T original) where T : ScriptableObject
    {
        T copy = (T)ScriptableObject.CreateInstance(original.GetType());
        JsonUtility.FromJsonOverwrite(JsonUtility.ToJson(original), copy);
        return copy;
    }
}
