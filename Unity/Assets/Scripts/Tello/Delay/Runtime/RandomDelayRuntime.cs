public class RandomDelayRuntime : DelayRuntime
{
    private MinMaxDelayConfig config;

    private float delay;

    public RandomDelayRuntime(MinMaxDelayConfig config)
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
        return UnityEngine.Random.Range(config.minDelay, config.maxDelay);
    }
}
