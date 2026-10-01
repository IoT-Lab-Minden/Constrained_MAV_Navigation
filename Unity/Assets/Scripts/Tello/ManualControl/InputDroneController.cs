using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

public abstract class InputDroneController : MonoBehaviour
{
    [Header("Drone")]
    public Drone drone;
    public bool isMainController = true;

    [Header("Controller Values")]
    public float yaw = 0.0f;
    public float elevation = 0.0f;
    public float roll = 0.0f;
    public float pitch = 0.0f;

    [Header("Settings")]
    [Range(0.0f, 1.0f)]
    public float droneSpeedMultiplier = 1.0f;
    [Range(0, 100)]
    public float updateRate = 20.0f;
    private float lastUpdateTime = 0.0f;

    private ControllerState lastState;

    public void Start()
    {
        if (drone == null)
        {
            GameObject droneObj = GameObject.FindGameObjectWithTag("Drone");
            if (droneObj != null)
                drone = droneObj.GetComponent<Drone>();
            else
                Debug.LogWarning("No drone selected for drone controller");
        }

        if (drone != null && isMainController)
            drone.StartConnecting();

        lastState = new ControllerState();
    }

    private void OnApplicationQuit()
    {
        if (isMainController)
            drone.StopConnecting();
    }

    private void FixedUpdate()
    {
        if (drone == null) return;

        // Handle takeoff and landing
        if (IsTakeoffRequested())
            drone.TakeOff();

        if (IsLandingRequested())
            drone.Land();

        if (Time.time - lastUpdateTime < 1.0f / updateRate)
            return; // Skip update if we haven't reached the next update time yet

        ControllerState state = GetNewControllerState();
        state.lx = state.lx * droneSpeedMultiplier;
        state.ly = state.ly * droneSpeedMultiplier;
        state.rx = state.rx * droneSpeedMultiplier;
        state.ry = state.ry * droneSpeedMultiplier;
        
        // Set values for visualization in inspector
        yaw = state.lx;
        elevation = state.ly;
        roll = state.rx;
        pitch = state.ry;

        // Check if we need to update controller state
        if (lastState == null || !state.Equals(lastState))
        {
            // Only update if we are flying
            if (drone.GetDroneState().flightState == FlightState.FLYING)
            {
                // Set actual controller values
                drone.SetControllerState(state);
                lastState = state;
            }
        }
        lastUpdateTime = Time.time;
    }

    protected abstract ControllerState GetNewControllerState();

    protected abstract bool IsTakeoffRequested();

    protected abstract bool IsLandingRequested();
}
