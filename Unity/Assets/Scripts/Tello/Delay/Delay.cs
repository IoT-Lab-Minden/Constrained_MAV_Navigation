using UnityEngine;

public abstract class Delay<T>
{
    private class DelayedPayload
    {
        public int generation;
        public T data;
    }

    private DelayConfig delayConfig;
    private DelayRuntime delayRuntime;
    private int generation = 0;

    protected Drone drone;

    public Delay(DelayConfig config, Drone drone)
    {
        delayConfig = config;
        this.drone = drone;
    }

    public void Init()
    {
        delayRuntime = delayConfig.CreateDelayRuntime();
        delayRuntime.Init();
    }

    public void Enqueue(T data)
    {
        float delayValue = GetDelayValue();
        DelayScheduler.Instance.ScheduleDelay(delayValue, new DelayedPayload { generation = generation, data = data }, OnDelayComplete);
    }

    public void ClearPending()
    {
        generation++;
    }

    public float GetDelayValue()
    {
        float delayValue = delayRuntime.GetDelayValue();
        return Mathf.Max(0f, delayValue); // Ensure non-negative delay
    }

    private void OnDelayComplete(object data)
    {
        DelayedPayload payload = (DelayedPayload)data;
        if (payload.generation != generation) return;

        ExecuteDelayedAction(payload.data);
    }

    protected abstract void ExecuteDelayedAction(T data);
}
