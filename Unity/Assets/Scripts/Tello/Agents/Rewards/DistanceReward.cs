using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using UnityEngine;

public class DistanceReward : Reward
{
    private float previousDistance = -1;
    private Waypoint currentWaypoint;

    public DistanceReward(float weight) : base(weight, "Distance")
    {
    }

    public override float CalculateReward(RLAgenticController agent, ActionBuffers actions, DroneState droneState, ParcoursState parcoursState)
    {
        // positive reward if the distance reduced compared to the last step, negative reward if it stayed the same or increased
        var nextWayPoint = parcoursState.currentWaypoint.transform.position;
        Vector3 previousWaypoint = parcoursState.previousWaypoint
            ? parcoursState.previousWaypoint.transform.position
            : parcoursState.startPosition;

        float segmentLength = Vector3.Distance(previousWaypoint, nextWayPoint);
        if (segmentLength < 1e-5f)
            segmentLength = 1f;

        var distanceToWaypoint = (droneState.position - nextWayPoint).magnitude;

        if (parcoursState.currentWaypoint != currentWaypoint)
        {
            currentWaypoint = parcoursState.currentWaypoint;
            previousDistance = distanceToWaypoint; // reset previous distance when reaching a new way point
            return 0f;
        }

        float distanceDelta = previousDistance - distanceToWaypoint;
        float normalizedDistance = Mathf.Clamp01(distanceToWaypoint / segmentLength);
        float proximityWeight = 1f + (1f - normalizedDistance);
        var distanceReward = (distanceDelta / segmentLength) * proximityWeight;

        previousDistance = distanceToWaypoint;

        statsRecorder.Add("Waypoint/AbsDistanceProgress", Mathf.Abs(distanceReward));
        statsRecorder.Add("Waypoint/DistanceProgress", distanceReward);
        
        episodeLogger.Add("goal/distance_progress", distanceReward);

        return distanceReward;
    }
}
