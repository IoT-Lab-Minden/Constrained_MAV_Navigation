using System;
using UnityEngine;

public class CBFKeyboardController : KeyboardController
{
    [Header("References")]
    public Parcours parcours;

    private CBFHandler cbfHandler = new CBFHandler();
    private int i = 0;

    protected override ControllerState GetNewControllerState()
    {
        DroneState droneState = new DroneState(
            drone.GetDroneState().flightState,
            drone.transform.position,
            drone.transform.rotation,
            drone.GetLinearVelocity(),
            drone.GetAngularVelocity(),
            drone.GetDroneSize(),
            i);
        i++;
        ParcoursState parcoursState = new ParcoursState(
            (ComplexWaypoint)parcours.GetCurrentWayPoint(),
            (ComplexWaypoint)parcours.GetPreviousWayPoint(),
            parcours.GetStartPosition(),
            false, 0, 0, false);
        ControllerState state = base.GetNewControllerState();
        cbfHandler.ApplyCBFs(ref state, droneState, parcoursState);
        return state;
    }
}
