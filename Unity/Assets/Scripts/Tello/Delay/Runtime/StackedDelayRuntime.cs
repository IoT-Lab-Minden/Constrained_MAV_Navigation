
public class StackedDelayRuntime : DelayRuntime
{
    private StackedDelayConfig config;
    private DelayRuntime[] delayRuntimes;

    public StackedDelayRuntime(StackedDelayConfig config)
    {
        this.config = config;
    }

    public override void Init()
    {
        delayRuntimes = new DelayRuntime[config.delayConfigs.Length];
        for (int i = 0; i < config.delayConfigs.Length; i++)
        {
            delayRuntimes[i] = config.delayConfigs[i].CreateDelayRuntime();
            delayRuntimes[i].Init();
        }
    }

    public override float GetDelayValue()
    {
        float totalDelay = 0f;
        foreach (var delayRuntime in delayRuntimes)
        {
            totalDelay += delayRuntime.GetDelayValue();
        }
        return totalDelay;
    }
}
