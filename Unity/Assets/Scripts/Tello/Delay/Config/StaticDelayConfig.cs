using UnityEngine;

[CreateAssetMenu(fileName = "StaticDelayConfig", menuName = "Drone/DelayConfig/StaticDelayConfig")]
public class StaticDelayConfig : DelayConfig
{
    public float delay = 0.0f;

    public override DelayRuntime CreateDelayRuntime()
    {
        return new StaticDelayRuntime(this);
    }
}
