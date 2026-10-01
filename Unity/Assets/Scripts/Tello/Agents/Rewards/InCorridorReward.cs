using Unity.MLAgents.Actuators;
using UnityEngine;

public class InCorridorReward : Reward
{
    public InCorridorReward(float weight) : base(weight, "InCorridor")
    {
    }

    public override float CalculateReward(RLAgenticController agent, ActionBuffers actions, DroneState droneState, ParcoursState parcoursState)
    {
        Vector3 currentWaypointPos = parcoursState.currentWaypoint ? parcoursState.currentWaypoint.transform.position : Vector3.zero;
        Vector3 previousWaypointPos = parcoursState.previousWaypoint ? parcoursState.previousWaypoint.transform.position : parcoursState.startPosition;
        
        var aToB = previousWaypointPos - currentWaypointPos;
        var aToDrone = droneState.position - currentWaypointPos;
        float segmentLength = aToB.magnitude;
        if (segmentLength < 1e-5f)
            return 0f;

        float distance = Vector3.Cross(aToDrone, aToB).magnitude / segmentLength;

        float corridorRadius = parcoursState.currentWaypoint.flightRadius;
    
        float x = distance / corridorRadius;
        return -x;
    }
}
