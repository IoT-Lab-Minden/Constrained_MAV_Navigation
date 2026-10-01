using UnityEngine;

[CreateAssetMenu(fileName = "GaussianDelayConfig", menuName = "Drone/DelayConfig/GaussianDelayConfig")]
public class GaussianDelayConfig : DelayConfig
{
    public float meanDelay = 0.0f;
    public float stdDev = 1.0f;

    public override DelayRuntime CreateDelayRuntime()
    {
        return new GaussianDelayRuntime(this);
    }
}
