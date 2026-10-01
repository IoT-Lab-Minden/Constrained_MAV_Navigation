
public class StaticDelayRuntime : DelayRuntime
{
    private StaticDelayConfig config;

    public StaticDelayRuntime(StaticDelayConfig config)
    {
        this.config = config;
    }

    public override float GetDelayValue()
    {
        return config.delay;
    }
}
