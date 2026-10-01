using Unity.MLAgents.Actuators;
using UnityEngine;

public class MaxSpeedReward : Reward
{
    public MaxSpeedReward(float weight) : base(weight, "MaxSpeed")
    {
    }

    public override float CalculateReward(RLAgenticController agent, ActionBuffers actions, DroneState droneState, ParcoursState parcoursState)
    {
        float speed = droneState.linearVelocity.magnitude;
        float maxSpeed = parcoursState.currentWaypoint.maxSpeed;

        if (maxSpeed <= 1e-5f)
            return 0f;

        float ratio = speed / maxSpeed;
        return -Mathf.Max(0f, ratio - 1f);
    }
}
