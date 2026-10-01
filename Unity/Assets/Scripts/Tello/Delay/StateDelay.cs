public class StateDelay : Delay<object>
{
    public StateDelay(DelayConfig config, Drone drone) : base(config, drone) { }

    protected override void ExecuteDelayedAction(object data)
    {
        drone.SetDroneState((DroneState)data);
    }
}
