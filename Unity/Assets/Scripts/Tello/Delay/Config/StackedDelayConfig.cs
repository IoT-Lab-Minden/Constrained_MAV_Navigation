using UnityEngine;

[CreateAssetMenu(fileName = "StackedDelayConfig", menuName = "Drone/DelayConfig/StackedDelayConfig")]
public class StackedDelayConfig : DelayConfig
{
    public DelayConfig[] delayConfigs;

    public override DelayRuntime CreateDelayRuntime()
    {
        return new StackedDelayRuntime(this);
    }
}
