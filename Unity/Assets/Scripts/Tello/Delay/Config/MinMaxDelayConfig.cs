using UnityEngine;

[CreateAssetMenu(fileName = "MinMaxDelayConfig", menuName = "Drone/DelayConfig/MinMaxDelayConfig")]
public class MinMaxDelayConfig : DelayConfig
{
    public float minDelay = 0.0f;
    public float maxDelay = 1.0f;

    public override DelayRuntime CreateDelayRuntime()
    {
        return new RandomDelayRuntime(this);
    }
}
