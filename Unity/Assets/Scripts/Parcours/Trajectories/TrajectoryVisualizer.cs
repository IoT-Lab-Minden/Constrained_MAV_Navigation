using System.Collections.Generic;
using UnityEngine;

[ExecuteAlways]
public class TrajectoryVisualizer : MonoBehaviour
{
    public Parcours parcours;
    public Drone drone;
    public ClassicAgenticController agent;

    // Caching to avoid regenerating the trajectory every Editor frame
    private Trajectory cachedGenerator = null;
    private int cachedSamplesPerSegment = 0;
    private Vector3 cachedDronePos = Vector3.zero;
    private Quaternion cachedDroneRot = Quaternion.identity;
    private float lastRegenerationTime = -999f;
    [Header("Caching thresholds")]
    [Tooltip("Position change (meters) that triggers regeneration")] public float dronePositionThreshold = 0.01f;
    [Tooltip("Rotation change (degrees) that triggers regeneration")] public float droneRotationThresholdDeg = 2.0f;
    [Tooltip("Minimum time (seconds) between regenerations to avoid excessive recomputes")] public float minRegenerationIntervalSec = 0.5f;

    [Header("Sampling")]
    [Range(4, 256)] public int samplesPerSegment = 48;
    [Header("Debug")]
    public bool debugPrintCoeffs = false;

    [Header("Visual")]
    public Color trajectoryColor = Color.cyan;
    public Color waypointColor = Color.yellow;
    public Color startColor = Color.green;
    public float waypointRadius = 0.05f;
    public float orientationLength = 0.25f;

    [Header("Time marker")]
    public Color timeMarkerColor = Color.magenta;
    public float timeMarkerRadius = 0.06f;

    void Start()
    {
        if (parcours == null) parcours = GetComponent<Parcours>();
    }

    void OnValidate()
    {
        if (parcours == null) parcours = GetComponent<Parcours>();
        if (samplesPerSegment < 1) samplesPerSegment = 1;
    }

    void OnDrawGizmos()
    {
        Trajectory generator;
        if (agent == null)
        {
            if (parcours == null) return;

            // Build and run minimum-snap generator quickly for visualization, but cache
            // the result and only regenerate when inputs change (drone pose thresholds, samples)
            // Also throttle regenerations to avoid excessive recomputes (max 2Hz).

            bool needRegen = false;
            if (cachedGenerator == null) needRegen = true;
            if (cachedSamplesPerSegment != samplesPerSegment) needRegen = true;
            if (drone != null)
            {
                // regenerate only when drone moved/rotated beyond thresholds
                if (Vector3.Distance(cachedDronePos, drone.transform.position) > dronePositionThreshold) needRegen = true;
                if (Quaternion.Angle(cachedDroneRot, drone.transform.rotation) > droneRotationThresholdDeg) needRegen = true;
            }
            else
            {
                // if previously we had cached drone pose but now drone is unset, regen
                if (cachedDronePos != Vector3.zero || cachedDroneRot != Quaternion.identity) needRegen = true;
            }

            // Throttle: only regenerate if minimum interval has passed
            if (needRegen && (Time.realtimeSinceStartup - lastRegenerationTime) >= minRegenerationIntervalSec)
            {
                OnlineMinSnapTrajectory newGenerator = new OnlineMinSnapTrajectory(parcours);
                if (drone != null) newGenerator.SetDrone(drone);
                // Always enable debug for trajectory generation in manual mode (agent == null)
                //MinimumSnapTrajectory.DebugPrint = true;
                newGenerator.GenerateTrajectory();
                //MinimumSnapTrajectory.DebugPrint = false;

                // Only update cache if generation was successful
                if (newGenerator.HasSolution())
                {
                    cachedGenerator = newGenerator;
                    cachedSamplesPerSegment = samplesPerSegment;
                    if (drone != null)
                    {
                        cachedDronePos = drone.transform.position;
                        cachedDroneRot = drone.transform.rotation;
                    }
                    else
                    {
                        cachedDronePos = Vector3.zero;
                        cachedDroneRot = Quaternion.identity;
                    }
                    lastRegenerationTime = (float)Time.realtimeSinceStartup;
                }
                // else: generation failed, keep old cachedGenerator (if any) for persistent display
            }
            
            generator = cachedGenerator;
        }
        else
        {
            generator = agent.cachedGenerator;
        }

        if (generator == null) return;

        // do not draw anything if no trajectory solution exists
        if (!generator.HasSolution())
        {
            return;
        }

        // Sample trajectory
        List<float> times = generator.GetSegmentTimes();
        if (times == null || times.Count < 2) return;

        Gizmos.color = trajectoryColor;
        Vector3 prev = generator.EvaluatePosition(times[0], false);

        for (int seg = 0; seg < times.Count - 1; seg++)
        {
            float t0 = times[seg];
            float t1 = times[seg + 1];
            for (int s = 1; s <= samplesPerSegment; s++)
            {
                float u = (float)s / (float)samplesPerSegment;
                float t = Mathf.Lerp(t0, t1, u);
                Vector3 p = generator.EvaluatePosition(t, false);
                Gizmos.DrawLine(prev, p);
                prev = p;

                // draw orientation arrow every few samples
                if (s % Mathf.Max(1, samplesPerSegment / 4) == 0)
                {
                    double psi = generator.EvaluateYaw(t, p, false);
                    Vector3 dir = new Vector3(Mathf.Sin((float)psi), 0f, Mathf.Cos((float)psi)).normalized;
                    Gizmos.color = Color.red;
                    Gizmos.DrawLine(p, p + dir * orientationLength);
                    // small arrow head
                    Vector3 right = Quaternion.LookRotation(dir) * Quaternion.Euler(0, 160, 0) * Vector3.forward;
                    Vector3 left = Quaternion.LookRotation(dir) * Quaternion.Euler(0, 200, 0) * Vector3.forward;
                    Gizmos.DrawLine(p + dir * orientationLength, p + dir * orientationLength + right * (orientationLength * 0.25f));
                    Gizmos.DrawLine(p + dir * orientationLength, p + dir * orientationLength + left * (orientationLength * 0.25f));
                    Gizmos.color = trajectoryColor;
                }
            }
        }

        // visualize the last time that was queried from the generator (if any)
        double lastT = generator.GetLastRequestedTime();
        if (!double.IsNaN(lastT))
        {
            var segTimes = generator.GetSegmentTimes();
            if (segTimes != null && segTimes.Count >= 2)
            {
                double segTstart = segTimes[0];
                double segTend = segTimes[segTimes.Count - 1];
                if (lastT >= segTstart && lastT <= segTend)
                {
                    Vector3 pos = generator.EvaluatePosition(lastT, false);
                    Gizmos.color = timeMarkerColor;
                    Gizmos.DrawSphere(pos, timeMarkerRadius);
                    Gizmos.DrawLine(pos, pos + Vector3.up * (timeMarkerRadius * 2f));
                }
            }
        }
    }
}
