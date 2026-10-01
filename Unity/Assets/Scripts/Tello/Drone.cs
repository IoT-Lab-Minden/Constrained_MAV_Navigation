using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public abstract class Drone : MonoBehaviour
{
    public enum DroneType { Tello, Simulation };

    public DroneType droneType;
    private float droneSize = 0.15f;
    
    [Header("State")]
    public FlightState flightState;

    [Header("Simulation")]
    [SerializeField]
    protected DelayConfig actionDelayConfig;
    [SerializeField]
    protected DelayConfig stateDelayConfig;
    [SerializeField]
    protected SimulationModel simulationModel;
    protected ActionDelay actionDelay;
    protected StateDelay stateDelay;

    public void Awake()
    {
        // Init delays
        actionDelay = new ActionDelay(actionDelayConfig, this);
        actionDelay.Init();
        stateDelay = new StateDelay(stateDelayConfig, this);
        stateDelay.Init();
    }

    public float GetDroneSize()
    {
        return droneSize;
    }

    public abstract bool IsConnected();
    public abstract void StartConnecting();
    public abstract void StopConnecting();

    public abstract void StartVideo();

    public abstract void TakeOff();
    public abstract void Land();

    public abstract void ResetDrone();
    public virtual void ResetDelays()
    {
        actionDelay.ClearPending();
        stateDelay.ClearPending();
    }

    public virtual void SetControllerState(ControllerState state)
    {
        SetControllerStateImmediate(state);
    }
    public void SetControllerState(float lx, float ly, float rx, float ry)
    {
        SetControllerState(new ControllerState(lx, ly, rx, ry));
    }
    public abstract void SetControllerStateImmediate(ControllerState state);

    public virtual void SetDroneState(DroneState state) { }

    public abstract DroneState GetDroneState();
    public virtual DroneState GetCurrentRealDroneState()
    {
        return GetDroneState();
    }

    public abstract Vector3 GetLinearVelocity();
    public abstract float GetAngularVelocity();

    public SimulationModel CreateSimulationModelClone()
    {
        return SimulationModel.Clone(simulationModel);
    }

    public float GetActionDelayEstimate()
    {
        return actionDelay.GetDelayValue();
    }

    public float GetStateDelayEstimate()
    {
        return stateDelay.GetDelayValue();
    }

    public abstract Pose GetPose();

    public class UpdateEvent : EventArgs
    {
        public UpdateEvent(int cmdId)
        {
            this.cmdId = cmdId;
        }

        public int cmdId { get; }
    }

    public class ByteDataEvent : EventArgs
    {
        public ByteDataEvent(byte[] data)
        {
            this.data = data;
        }

        public byte[] data { get; }
    }

    public class StampedByteDataEvent : EventArgs
    {
        public StampedByteDataEvent(DateTime timestamp, byte[] data)
        {
            this.timestamp = timestamp;
            this.data = data;
        }

        public DateTime timestamp { get; }
        public byte[] data { get; }
    }

    public class ConnectionStateEvent : EventArgs
    {
        public ConnectionStateEvent(ConnectionState connectionState)
        {
            this.connectionState = connectionState;
        }

        public ConnectionState connectionState { get; }
    }

    public class StampedControllerStateEvent : EventArgs
    {
        public StampedControllerStateEvent(DateTime timestamp, ControllerState state)
        {
            this.timestamp = timestamp;
            this.state = state;
        }

        public DateTime timestamp { get; }
        public ControllerState state { get; }
    }

    public event EventHandler<UpdateEvent> OnUpdate;
    public event EventHandler<StampedByteDataEvent> OnRawUpdate;
    public event EventHandler<ConnectionStateEvent> OnConnection;
    public event EventHandler<ByteDataEvent> OnVideoData;
    public event EventHandler<StampedByteDataEvent> OnRawVideoData;

    public event EventHandler<StampedControllerStateEvent> OnControllerUpdate;
    public event EventHandler OnTakeoff;
    public event EventHandler OnLand;

    public void CallOnUpdate(UpdateEvent e)
    {
        OnUpdate?.Invoke(this, e);
    }
    public void CallOnRawUpdate(StampedByteDataEvent e)
    {
        OnRawUpdate?.Invoke(this, e);
    }
    public void CallOnConnection(ConnectionStateEvent e)
    {
        OnConnection?.Invoke(this, e);
    }
    public void CallOnVideoData(ByteDataEvent e)
    {
        OnVideoData?.Invoke(this, e);
    }
    public void CallOnRawVideoData(StampedByteDataEvent e)
    {
        OnRawVideoData?.Invoke(this, e);
    }

    public void CallOnControllerUpdate(StampedControllerStateEvent e)
    {
        OnControllerUpdate?.Invoke(this, e);
    }
    public void CallOnTakeoff(EventArgs e)
    {
        OnTakeoff?.Invoke(this, e);
    }
    public void CallOnLand(EventArgs e)
    {
        OnLand?.Invoke(this, e);
    }


    public enum ConnectionState
    {
        Disconnected,
        Connecting,
        Connected,
        Paused,//used to keep from disconnecting when starved for input.
        UnPausing//Transition. Never stays in this state.
    }
}
