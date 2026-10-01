using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using UnityEngine;
using System.Collections.Generic;

public class SummedReward : Reward
{
    private Reward[] rewards;

    public SummedReward(string name, params Reward[] rewards) : this(1.0f, name, rewards) { }

    public SummedReward(float weight, string name, params Reward[] rewards) : base(weight, name)
    {
        this.rewards = rewards;
    }

    public override void SetLoggers(StatsRecorder statsRecorder, EpisodeLogger episodeLogger)
    {
        base.SetLoggers(statsRecorder, episodeLogger);
        foreach (var reward in rewards)
        {
            reward.SetLoggers(statsRecorder, episodeLogger);
        }
    }

    public override float CalculateReward(RLAgenticController agent, ActionBuffers actions, DroneState droneState, ParcoursState parcoursState)
    {
        float sum = 0f;
        List<float> individualRewards = new List<float>();
        foreach (var reward in rewards)
        {
            float rewardValue = reward.GetReward(agent, actions, droneState, parcoursState);
            individualRewards.Add(rewardValue);
            sum += rewardValue;
        }
        statsRecorder.Add("Reward/"+name, sum, StatAggregationMethod.Histogram);
        
        return sum;
    }

    public override void ResetRewardOnWaypointReached()
    {
        foreach (var reward in rewards)
        {
            reward.ResetRewardOnWaypointReached();
        }
    }
}
