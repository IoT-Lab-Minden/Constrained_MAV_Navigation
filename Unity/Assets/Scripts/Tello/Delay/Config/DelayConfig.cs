using UnityEngine;

public abstract class DelayConfig
 : ScriptableObject
{
    public bool fixedDelay;

    public abstract DelayRuntime CreateDelayRuntime();
}
