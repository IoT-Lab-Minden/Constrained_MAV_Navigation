using System;
using System.Collections.Generic;
using System.Net.Http;
using System.Text;
using UnityEngine;
using System.Collections;

public class OnlineMinSnapTrajectory : Trajectory
{
    private const string GenerateUrl = "http://127.0.0.1:5000/generate";
    private const float DefaultCorridorWidth = 1.00f;

    // Stored results
    private List<double[]> coeffsPerSegmentX;
    private List<double[]> coeffsPerSegmentY;
    private List<double[]> coeffsPerSegmentZ;
    private List<double[]> coeffsPerSegmentPsi;
    private List<float> segmentTimes;

    [Serializable]
    private class RequestPayload
    {
        public List<RequestWaypoint> waypoints;
    }

    [Serializable]
    private class RequestWaypoint
    {
        public float time;
        public float[] position;
        public float yaw;
        public float corridor_width;
        public float max_speed;
    }

    [Serializable]
    private class ResponseSegmentListWrapper
    {
        public ResponseSegment[] items;
    }

    [Serializable]
    private class ResponseSegment
    {
        public float segment_time;
        public PositionCoefficients position_coeffs;
        public double[] yaw_coeffs;
    }

    [Serializable]
    private class PositionCoefficients
    {
        public double[] x;
        public double[] y;
        public double[] z;
    }

    public OnlineMinSnapTrajectory(Parcours parcours, float maxSpeed = 1.1F) : base(parcours, maxSpeed)
    {
    }

    public override void GenerateTrajectory()
    {
        coeffsPerSegmentX = null;
        coeffsPerSegmentY = null;
        coeffsPerSegmentZ = null;
        coeffsPerSegmentPsi = null;
        segmentTimes = null;

        var waypointList = parcours.GetWaypointList();
        var positions = GetKeyframePositions(waypointList);
        if (positions == null || positions.Count < 2)
        {
            Debug.LogWarning("OnlineMinSnapTrajectory: Not enough keyframes to generate trajectory.");
            return;
        }

        var times = ComputeKeyframeTimes(waypointList);
        var yawList = ComputeYawList(waypointList);
        if (times == null || yawList == null || times.Count != positions.Count || yawList.Count != positions.Count)
        {
            Debug.LogWarning("OnlineMinSnapTrajectory: Invalid keyframe data (times/yaw mismatch).");
            return;
        }

        var payload = new RequestPayload
        {
            waypoints = new List<RequestWaypoint>(positions.Count)
        };

        int offset = (droneRef != null) ? 1 : 0;
        for (int i = 0; i < positions.Count; i++)
        {
            float corridorWidth = DefaultCorridorWidth;
            float waypointMaxSpeed = maxSpeed;

            int wpIndex = i - offset;
            if (wpIndex >= 0 && wpIndex < waypointList.Count && waypointList[wpIndex] is ComplexWaypoint complexWp)
            {
                corridorWidth = Mathf.Min(DefaultCorridorWidth, complexWp.flightRadius);
                waypointMaxSpeed = Mathf.Min(maxSpeed, complexWp.maxSpeed);
            }

            var reqWp = new RequestWaypoint
            {
                time = times[i],
                position = new[] { positions[i].x, positions[i].y, positions[i].z },
                yaw = yawList[i],
                corridor_width = corridorWidth,
                max_speed = waypointMaxSpeed
            };

            payload.waypoints.Add(reqWp);
        }

        parcours.StartCoroutine(RequestTrajectoryCoefficients(payload));
    }

    private IEnumerator RequestTrajectoryCoefficients(RequestPayload payload)
    {
        string requestJson = JsonUtility.ToJson(payload);
        string responseJson = null;
        Debug.Log(requestJson);

        using (var client = new HttpClient())
        {
            client.Timeout = TimeSpan.FromSeconds(120);
            using (var requestBody = new StringContent(requestJson, Encoding.UTF8, "application/json"))
            {
                var postTask = client.PostAsync(GenerateUrl, requestBody);
                while (!postTask.IsCompleted)
                {
                    yield return null;
                }

                if (postTask.IsFaulted || postTask.IsCanceled)
                {
                    Debug.LogWarning($"OnlineMinSnapTrajectory: Request task failed: {postTask.Exception?.GetBaseException().Message}");
                    yield break;
                }

                var response = postTask.Result;
                if (!response.IsSuccessStatusCode)
                {
                    Debug.LogWarning($"OnlineMinSnapTrajectory: Server returned HTTP {(int)response.StatusCode} ({response.ReasonPhrase}).");
                    yield break;
                }

                var readTask = response.Content.ReadAsStringAsync();
                while (!readTask.IsCompleted)
                {
                    yield return null;
                }

                if (readTask.IsFaulted || readTask.IsCanceled)
                {
                    Debug.LogWarning($"OnlineMinSnapTrajectory: Failed reading response content: {readTask.Exception?.GetBaseException().Message}");
                    yield break;
                }

                responseJson = readTask.Result;
            }
        }

        if (responseJson == null)
        {
            yield break;
        }
        Debug.Log(responseJson);

        ResponseSegment[] segments = null;
        try
        {
            string wrapped = "{\"items\":" + responseJson + "}";
            var wrapper = JsonUtility.FromJson<ResponseSegmentListWrapper>(wrapped);
            segments = wrapper?.items;
        }
        catch (Exception ex)
        {
            Debug.LogWarning($"OnlineMinSnapTrajectory: Failed to parse server response: {ex.Message}");
        }

        if (segments == null || segments.Length == 0)
        {
            Debug.LogWarning("OnlineMinSnapTrajectory: Server returned no trajectory segments.");
            yield return null;
        }

        coeffsPerSegmentX = new List<double[]>(segments.Length);
        coeffsPerSegmentY = new List<double[]>(segments.Length);
        coeffsPerSegmentZ = new List<double[]>(segments.Length);
        coeffsPerSegmentPsi = new List<double[]>(segments.Length);
        segmentTimes = new List<float>(segments.Length + 1) { 0f };

        float accumulatedTime = 0f;
        for (int i = 0; i < segments.Length; i++)
        {
            var seg = segments[i];
            if (seg == null || seg.position_coeffs == null || seg.position_coeffs.x == null || seg.position_coeffs.y == null || seg.position_coeffs.z == null || seg.yaw_coeffs == null)
            {
                Debug.LogWarning($"OnlineMinSnapTrajectory: Invalid segment data at index {i}.");
                coeffsPerSegmentX = null;
                coeffsPerSegmentY = null;
                coeffsPerSegmentZ = null;
                coeffsPerSegmentPsi = null;
                segmentTimes = null;
                yield return null;
            }

            coeffsPerSegmentX.Add(seg.position_coeffs.x);
            coeffsPerSegmentY.Add(seg.position_coeffs.y);
            coeffsPerSegmentZ.Add(seg.position_coeffs.z);
            coeffsPerSegmentPsi.Add(seg.yaw_coeffs);

            float dt = Mathf.Max(MinSegmentDuration, seg.segment_time);
            accumulatedTime += dt;
            segmentTimes.Add(accumulatedTime);
        }

        if (segmentTimes.Count != coeffsPerSegmentX.Count + 1)
        {
            Debug.LogWarning("OnlineMinSnapTrajectory: Segment time count mismatch.");
            coeffsPerSegmentX = null;
            coeffsPerSegmentY = null;
            coeffsPerSegmentZ = null;
            coeffsPerSegmentPsi = null;
            segmentTimes = null;
        }
    }

    public override bool HasSolution()
    {
        return coeffsPerSegmentX != null &&
               coeffsPerSegmentY != null &&
               coeffsPerSegmentZ != null &&
               coeffsPerSegmentPsi != null &&
               segmentTimes != null &&
               segmentTimes.Count >= 2;
    }

    public override List<float> GetSegmentTimes()
    {
        return (segmentTimes == null) ? new List<float>() : new List<float>(segmentTimes);
    }

    public override Vector3 EvaluatePosition(double t, bool record = true)
    {
        if (!HasSolution()) return Vector3.zero;
        if (record) lastRequestedTime = t;

        int m = segmentTimes.Count;
        if (t <= segmentTimes[0]) t = segmentTimes[0];
        if (t >= segmentTimes[m - 1]) t = segmentTimes[m - 1];

        int seg = 0;
        for (int i = 0; i < m - 1; i++)
        {
            if (t >= segmentTimes[i] && t <= segmentTimes[i + 1])
            {
                seg = i;
                break;
            }
        }

        double t0 = segmentTimes[seg];
        double tau = t - t0;

        double x = EvalPolyAt(coeffsPerSegmentX[seg], tau);
        double y = EvalPolyAt(coeffsPerSegmentY[seg], tau);
        double z = EvalPolyAt(coeffsPerSegmentZ[seg], tau);

        return new Vector3((float)x, (float)y, (float)z);
    }

    public override double EvaluateYaw(double t, Vector3 position, bool record = true)
    {
        if (!HasSolution()) return 0.0;
        if (record) lastRequestedTime = t;

        if (position != Vector3.negativeInfinity) {
            var waypointList = parcours.GetWaypointList();
            int seg = 0;
            for (int i = 0; i < segmentTimes.Count - 1; i++)
            {
                if (t >= segmentTimes[i] && t <= segmentTimes[i + 1])
                {
                    seg = i;
                    break;
                }
            }
            
            Vector3 dir = Vector3.forward;
            if (seg < waypointList.Count)
            {
                Waypoint wp = waypointList[seg];
                if (wp is ComplexWaypoint cw)
                {
                    switch (cw.droneOrientationType)
                    {
                        case DroneOrientationType.STATIC:
                            dir = cw.droneOrientation;
                            break;
                        case DroneOrientationType.TOWARDS_POINT:
                            dir = cw.droneOrientation - position;
                            break;
                        case DroneOrientationType.TOWARDS_WAYPOINT:
                            dir = cw.transform.position - position;
                            break;
                        default:
                            dir = cw.transform.forward;
                            break;
                    }
                }
            }

            dir.y = 0f;
            if (dir.sqrMagnitude < 1e-6f)
            {
                // fallback
                dir = Vector3.forward;
            }

            return Mathf.Atan2(dir.x, dir.z);
        } else {
            int m = segmentTimes.Count;
            if (t <= segmentTimes[0]) t = segmentTimes[0];
            if (t >= segmentTimes[m - 1]) t = segmentTimes[m - 1];

            int seg = 0;
            for (int i = 0; i < m - 1; i++)
            {
                if (t >= segmentTimes[i] && t <= segmentTimes[i + 1])
                {
                    seg = i;
                    break;
                }
            }

            double t0 = segmentTimes[seg];
            double tau = t - t0;

            return EvalPolyAt(coeffsPerSegmentPsi[seg], tau);
        }
    }
}
