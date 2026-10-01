using UnityEngine;

public struct ParcoursState
{
    public ComplexWaypoint currentWaypoint;
    public ComplexWaypoint previousWaypoint;
    public Vector3 startPosition;
    public bool hitWaypoint;
    public int stepCount;
    public int lastWaypointHit;
    public bool cbfTriggered;

    public ParcoursState(ComplexWaypoint currentWaypoint, ComplexWaypoint previousWaypoint, Vector3 startPosition, bool hitWaypoint, int stepCount, int lastWaypointHit, bool cbfTriggered)
    {
        this.currentWaypoint = currentWaypoint;
        this.previousWaypoint = previousWaypoint;
        this.startPosition = startPosition;
        this.hitWaypoint = hitWaypoint;
        this.stepCount = stepCount;
        this.lastWaypointHit = lastWaypointHit;
        this.cbfTriggered = cbfTriggered;
    }
}
