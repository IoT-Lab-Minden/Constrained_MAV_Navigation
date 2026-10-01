using System;
using System.Collections.Generic;
using UnityEngine;

public class ComplexWaypoint : Waypoint
{
    [Header("Drone requirements")]
    public float flightRadius = 0.05f;
    public float maxSpeed = 1.0f;
    public DroneOrientationType droneOrientationType;
    public Vector3 droneOrientation;

    private List<GameObject> closeObjects;

    // Fixes bug: When the previous waypoint is very close and this waypoint is
    // really large, the drone can get stuck at this waypoint, when the waypoint
    // got activated with the drone inside.
    private List<Collider> droneObjects;

    void Awake()
    {
        closeObjects = new List<GameObject>();
        droneObjects = new List<Collider>();
    }

    void Start () {
        SetTriggerRadius(flightRadius);
    }

    public void SetDroneRequirements(float flightRadius, float maxSpeed, DroneOrientationType droneOrientationType, Vector3 droneOrientation) {
        this.flightRadius = flightRadius;
        SetTriggerRadius(flightRadius);
        this.maxSpeed = maxSpeed;
        this.droneOrientationType = droneOrientationType;
        this.droneOrientation = droneOrientation;
    }

    public override void SetTriggerRadius(float triggerRadius) {
        GetComponent<SphereCollider>().radius = triggerRadius;
        GetComponent<BoxCollider>().size = new Vector3(0.04f, 2*triggerRadius, 2*triggerRadius);
        transform.GetChild(0).localScale = new Vector3(2*triggerRadius, 0.02f, 2*triggerRadius);
    }

    private void OnTriggerEnter(Collider other) {
        if (!closeObjects.Contains(other.gameObject)) {
            // Object has only entered a single collider
            closeObjects.Add(other.gameObject);
        } else {
            // Object has entered both colliders
            if (other.gameObject.tag != "Drone") return;

            // if (Vector3.Angle(other.gameObject.GetComponent<Drone>().GetLinearVelocity(), transform.forward) > 90) return;

            CallOnWaypointHit(new WaypointReachedEvent(other));
            if (state != WaypointState.ACTIVE)
            {
                droneObjects.Add(other);
                return;
            }

            state = WaypointState.COMPLETED;
            UpdateMaterial();
            CallOnWaypointReached(new WaypointReachedEvent(other));
        }
    }

    public override void OnWaypointActivated()
    {
        if (droneObjects == null) return;

        for (int i = droneObjects.Count - 1; i >= 0; i--)
        {
            Collider other = droneObjects[i];
            state = WaypointState.COMPLETED;
            UpdateMaterial();
            
            droneObjects.RemoveAt(i);
            
            CallOnWaypointHit(new WaypointReachedEvent(other));
            CallOnWaypointReached(new WaypointReachedEvent(other));
        }
    }

    private void OnTriggerExit(Collider other)
    {
        closeObjects.Remove(other.gameObject);
        droneObjects.Remove(other);
    }
}
