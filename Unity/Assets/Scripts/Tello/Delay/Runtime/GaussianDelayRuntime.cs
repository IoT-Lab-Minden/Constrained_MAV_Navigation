using UnityEngine;

public class GaussianDelayRuntime : DelayRuntime
{
    private GaussianDelayConfig config;

    private float delay;

    public GaussianDelayRuntime(GaussianDelayConfig config)
    {
        this.config = config;
    }

    public override void Init()
    {
        if (config.fixedDelay)
        {
            delay = CalculateDelay();
        }
    }

    public override float GetDelayValue()
    {
        if (config.fixedDelay)
        {
            return delay;
        }

        return CalculateDelay();
    }

    private float CalculateDelay()
    {
        // Generate a random delay value based on the Gaussian distribution parameters
        float mean = config.meanDelay;
        float stdDev = config.stdDev;

        // Box-Muller transform to generate a normally distributed random value
        float u1 = UnityEngine.Random.value; // Uniform(0,1) random value
        float u2 = UnityEngine.Random.value; // Uniform(0,1) random value

        float z0 = Mathf.Sqrt(-2.0f * Mathf.Log(u1)) * Mathf.Cos(2.0f * Mathf.PI * u2);
        return mean + z0 * stdDev; // Return the generated delay value
    }
}
