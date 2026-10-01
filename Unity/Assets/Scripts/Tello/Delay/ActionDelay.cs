public class ActionDelay : Delay<ControllerState>
{
    public ActionDelay(DelayConfig config, Drone drone) : base(config, drone) { }

    protected override void ExecuteDelayedAction(ControllerState state)
    {
        drone.SetControllerStateImmediate(state);
    }
}
