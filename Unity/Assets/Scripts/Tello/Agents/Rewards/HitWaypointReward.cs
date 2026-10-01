using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using UnityEngine;

public class HitWaypointReward : Reward
{
    public HitWaypointReward(float weight) : base(weight, "HitWaypoint")
    {
    }

    public override float CalculateReward(RLAgenticController agent, ActionBuffers actions, DroneState droneState, ParcoursState parcoursState)
    {
        float hitReward = parcoursState.hitWaypoint ? 1 : 0;
        if (parcoursState.hitWaypoint) {
            
            statsRecorder.Add("Waypoint/StepsRequired", (parcoursState.stepCount - parcoursState.lastWaypointHit) < 1500 ? (parcoursState.stepCount - parcoursState.lastWaypointHit) : 1500, StatAggregationMethod.Histogram);
            episodeLogger.Add("episode/waypoint_hit", 1);
            episodeLogger.Add("episode/steps_required", parcoursState.stepCount - parcoursState.lastWaypointHit);
        }

        return hitReward;
    }
}
