using UnityEngine;

public struct DroneState
{
    public FlightState flightState;
    public Vector3 position;
    public Quaternion rotation;
    public Vector3 linearVelocity;
    public float angularVelocity;

    public float droneSize;
    public long sequenceId;
    
    //
    // Summary:
    //     Returns the forward vector of the pose.
    public Vector3 forward => rotation * Vector3.forward;

    //
    // Summary:
    //     Returns the right vector of the pose.
    public Vector3 right => rotation * Vector3.right;

    //
    // Summary:
    //     Returns the up vector of the pose.
    public Vector3 up => rotation * Vector3.up;

    public DroneState(FlightState flightState, Vector3 position, Quaternion rotation, Vector3 linearVelocity, float angularVelocity, float droneSize, long sequenceId = 0)
    {
        this.flightState = flightState;
        this.position = position;
        this.rotation = rotation;
        this.linearVelocity = linearVelocity;
        this.angularVelocity = angularVelocity;
        this.droneSize = droneSize;
        this.sequenceId = sequenceId;
    }
}
