using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;

public class CBFHandler
{

    private List<ControlBarrierFunction> cbfs = new List<ControlBarrierFunction>();

    public CBFHandler() {
        // instantiate defaults if none configured in inspector
        if (cbfs == null || cbfs.Count == 0)
        {
            cbfs = new List<ControlBarrierFunction>
            {
                new MaxSpeedCBF(),
                new OrientationCBF(),
                new CorridorCBF()
                // new CorridorInputCBF()
            };
        }
    }

    public void SetLoggers(StatsRecorder statsRecorder, EpisodeLogger episodeLogger)
    {
        foreach (var cbf in cbfs)
        {
            cbf.SetLoggers(statsRecorder, episodeLogger);
        }
    }

    public bool ApplyCBFs(ref ControllerState controllerState, DroneState drone, ParcoursState parcours)
    {
        bool modified = false;
        foreach (var cbf in cbfs)
        {
            modified |= cbf.ApplyCBF(ref controllerState, drone, parcours);
        }
        return modified;
    }
}
