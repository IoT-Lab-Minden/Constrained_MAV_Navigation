using Unity.MLAgents;

public abstract class ControlBarrierFunction
{
    protected float safetyMargin = 0.0f;

    protected StatsRecorder statsRecorder;
    public EpisodeLogger episodeLogger;
    
    public void SetSafetyMargin(float margin)
    {
        safetyMargin = margin;
    }

    public void SetLoggers(StatsRecorder statsRecorder, EpisodeLogger episodeLogger)
    {
        this.statsRecorder = statsRecorder;
        this.episodeLogger = episodeLogger;
    }

    public abstract bool ApplyCBF(ref ControllerState controllerState, DroneState drone, ParcoursState parcours);
}
