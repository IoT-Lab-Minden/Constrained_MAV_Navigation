using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public abstract class ParcoursGenerator
{
    public enum ParcoursType {
        STATIC,
        CIRCLE,
        RANDOM
    }
    protected Parcours parent;
    protected float size;
    protected int waypointCount;
    protected Waypoint.Type waypointType;
    protected Action<object, Waypoint.WaypointReachedEvent> waypointCallback;
    protected Action<object, Waypoint.WaypointReachedEvent> waypointHitCallback;
    protected ParcoursType type;

    public ParcoursGenerator(Parcours parent, float size, int waypointCount, Waypoint.Type waypointType, Action<object, Waypoint.WaypointReachedEvent> waypointCallback, Action<object, Waypoint.WaypointReachedEvent> waypointHitCallback) {
        this.parent = parent;
        this.size = size;
        this.waypointCount = waypointCount;
        this.waypointType = waypointType;
        this.waypointCallback = waypointCallback;
        this.waypointHitCallback = waypointHitCallback;
    }
    public abstract List<Waypoint> Generate(Vector3 startPosition);

    public ParcoursType GetParcoursType() {
        return type;
    }

    protected void ClearWaypointObjects() {
        for(int i = parent.transform.childCount-1; i >= 0; i--) {
            GameObject.Destroy(parent.transform.GetChild(i).gameObject);
        }
    }

    protected Waypoint InstantiateWaypoint(Vector3 position) {
        return InstantiateWaypoint(position, Quaternion.identity);
    }

    protected Waypoint InstantiateWaypoint(Vector3 position, Quaternion orientation) {
        Waypoint waypoint = WaypointBuilder.GenerateWaypoint(waypointType);
        waypoint.SetTransformData(parent, position, orientation);
        waypoint.SetWaypointHitCallback(waypointHitCallback);
        waypoint.SetWaypointReachedCallback(waypointCallback);

        waypoint.ResetWaypoint();

        return waypoint;
    }
}
