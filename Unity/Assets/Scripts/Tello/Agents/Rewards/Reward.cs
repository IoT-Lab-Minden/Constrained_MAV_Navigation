using System.Text.RegularExpressions;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;

public abstract class Reward
{
    public string name;
    protected float weight;

    protected StatsRecorder statsRecorder;
    protected EpisodeLogger episodeLogger;

    public Reward(float weight, string name)
    {
        this.name = name;
        this.weight = weight;
    }

    public virtual void SetLoggers(StatsRecorder statsRecorder, EpisodeLogger episodeLogger)
    {
        this.statsRecorder = statsRecorder;
        this.episodeLogger = episodeLogger;
    }

    public float GetReward(RLAgenticController agent, ActionBuffers actions, DroneState droneState, ParcoursState parcoursState) {
        float reward = weight * CalculateReward(agent, actions, droneState, parcoursState);

        statsRecorder.Add("Reward/"+name, reward);
        episodeLogger.Add("reward/"+ToSnakeCase(name), reward);

        return reward;
    }

    public abstract float CalculateReward(RLAgenticController agent, ActionBuffers actions, DroneState droneState, ParcoursState parcoursState);

    public virtual void ResetRewardOnWaypointReached() {}

    protected string ToSnakeCase(string s)
    {
        return Regex.Replace(s, "(?<!^)([A-Z])", "_$1").ToLower();
    }
}
