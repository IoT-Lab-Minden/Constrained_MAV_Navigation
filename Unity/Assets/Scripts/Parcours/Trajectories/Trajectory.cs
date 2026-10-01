using System.Collections.Generic;
using UnityEngine;

public abstract class Trajectory
{
    protected Parcours parcours;
    // maximum allowed speed (m/s) used to compute segment times when not provided
    protected float maxSpeed;
    // optional drone reference used to include start position as first keyframe
    protected Drone droneRef;
    
    // record the last time value that was requested via EvaluatePosition/EvaluateYaw
    protected double lastRequestedTime = double.NaN;

    // Minimum segment duration to avoid zero-length segments (seconds)
    protected const float MinSegmentDuration = 0.01f;

    public Trajectory(Parcours parcours)
    {
        this.parcours = parcours;
    }

    public Trajectory(Parcours parcours, float maxSpeed=1.10f)
    {
        this.parcours = parcours;
        this.maxSpeed = maxSpeed;
    }

    /// <summary>
    /// Set the drone whose position will be used as the start keyframe when computing trajectories.
    /// </summary>
    public void SetDrone(Drone drone)
    {
        this.droneRef = drone;
    }

    /// <summary>
    /// Compute keyframe times proportional to inter-waypoint distances using the trajectory's maxSpeed.
    /// Returns a list of times (t0 = 0) with length equal to waypointList.Count.
    /// </summary>
    protected List<float> ComputeKeyframeTimes(List<Waypoint> waypointList)
    {
        var times = new List<float>();
        // build position list including drone start if available
        var positions = GetKeyframePositions(waypointList);
        if (positions == null || positions.Count == 0) return times;

        times.Add(0f);
        for (int i = 1; i < positions.Count; i++)
        {
            float dist = Vector3.Distance(positions[i - 1], positions[i]);
            float dt = (maxSpeed > 0f) ? dist / maxSpeed : MinSegmentDuration;
            if (dt < MinSegmentDuration) dt = MinSegmentDuration;
            times.Add(times[i - 1] + dt);
        }

        return times;
    }

    /// <summary>
    /// Compute yaw (radians) for each waypoint. For ComplexWaypoint the orientation is taken
    /// from the ComplexWaypoint fields
    /// </summary>
    protected List<float> ComputeYawList(List<Waypoint> waypointList)
    {
        var yaws = new List<float>();
        var positions = GetKeyframePositions(waypointList);
        if (positions == null || positions.Count == 0) return yaws;

        // offset for mapping waypointList indices into positions (1 if droneRef present)
        int offset = (droneRef != null) ? 1 : 0;

        for (int idx = 0; idx < positions.Count; idx++)
        {
            Vector3 dir = Vector3.zero;
            if (idx == 0 && droneRef != null)
            {
                // for start keyframe, look to next waypoint if exists, otherwise use drone forward
                //if (positions.Count > 1)
                //    dir = positions[1] - positions[0];
                //else
                    dir = Quaternion.Euler(0f, 90f, 0f) * droneRef.transform.forward;
            }
            else
            {
                int wpIndex = idx - offset;
                if (wpIndex < 0 || waypointList == null || wpIndex >= waypointList.Count)
                {
                    dir = Vector3.forward;
                }
                else
                {
                    Waypoint wp = waypointList[wpIndex];
                    if (wp is ComplexWaypoint cw)
                    {
                        switch (cw.droneOrientationType)
                        {
                            case DroneOrientationType.STATIC:
                                dir = cw.droneOrientation;
                                break;
                            case DroneOrientationType.TOWARDS_POINT:
                                dir = cw.droneOrientation - cw.transform.position;
                                break;
                            case DroneOrientationType.TOWARDS_WAYPOINT:
                                // determine next position in global positions list
                                int nextIdx = idx + 1;
                                if (nextIdx < positions.Count)
                                    dir = positions[nextIdx] - cw.transform.position;
                                else
                                    dir = cw.transform.forward;
                                break;
                            default:
                                dir = cw.transform.forward;
                                break;
                        }
                    }
                }
            }

            dir.y = 0f;
            if (dir.sqrMagnitude < 1e-6f)
            {
                // fallback
                dir = Vector3.forward;
            }

            float yaw = Mathf.Atan2(dir.x, dir.z);
            yaws.Add(yaw);
        }

        return yaws;
    }

    /// <summary>
    /// Return a list of positions that represent the keyframes. If a drone was set via SetDrone,
    /// its position is prepended as the first keyframe.
    /// </summary>
    protected List<Vector3> GetKeyframePositions(List<Waypoint> waypointList)
    {
        var positions = new List<Vector3>();
        if (droneRef != null)
        {
            positions.Add(droneRef.transform.position);
        }
        if (waypointList != null)
        {
            for (int i = 0; i < waypointList.Count; i++) positions.Add(waypointList[i].transform.position);
        }
        return positions;
    }

    public abstract void GenerateTrajectory();

    /// <summary>
    /// Returns the last time that was queried on this trajectory (or NaN if none).
    /// </summary>
    public double GetLastRequestedTime()
    {
        return lastRequestedTime;
    }

    // Public helpers for visualization/evaluation
    public abstract bool HasSolution();

    public abstract List<float> GetSegmentTimes();

    public float GetTotalTime()
    {
        var times = GetSegmentTimes();
        if (times == null || times.Count == 0) return 0f;
        return times[times.Count - 1];
    }

    /// <summary>
    /// Evaluate position (x,y,z) at global time t (clamped to trajectory bounds).
    /// </summary>
    public abstract Vector3 EvaluatePosition(double t, bool record = true);

    /// <summary>
    /// Evaluate yaw (psi) at global time t (radians).
    /// </summary>
    public abstract double EvaluateYaw(double t, Vector3 position, bool record = true);
    

    protected static double EvalPolyAt(double[] coeffs, double s)
    {
        if (coeffs == null) return 0.0;
        // Horner evaluation from highest degree to lowest
        double res = 0.0;
        for (int i = coeffs.Length - 1; i >= 0; i--)
        {
            res = res * s + coeffs[i];
        }
        return res;
    }
}
