using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using UnityEngine;

public class OrientationReward : Reward
{
    private float previousTheta = -1;
    private Waypoint currentWaypoint;

    public OrientationReward(float weight) : base(weight, "Orientation")
    {
    }

    public override float CalculateReward(RLAgenticController agent, ActionBuffers actions, DroneState droneState, ParcoursState parcoursState)
    {            
        Vector3 droneOrientation = Quaternion.AngleAxis(90, Vector3.up) * droneState.forward;
        droneOrientation.y = 0;
        
        Vector3 desiredOrientation = Vector3.zero;
        string loggingKey = "Orientation/Undefined";
        switch (parcoursState.currentWaypoint.droneOrientationType)
        {
            case DroneOrientationType.TOWARDS_WAYPOINT:
                desiredOrientation = Vector3.Normalize(parcoursState.currentWaypoint.transform.position - droneState.position);
                loggingKey = "Orientation/TowardWaypoint";
                break;
            case DroneOrientationType.TOWARDS_POINT:
                desiredOrientation = Vector3.Normalize(parcoursState.currentWaypoint.droneOrientation - droneState.position);
                loggingKey = "Orientation/TowardPoint";
                break;
            case DroneOrientationType.STATIC:
                desiredOrientation = parcoursState.currentWaypoint.droneOrientation;
                loggingKey = "Orientation/Static";
                break;
        }
        desiredOrientation.y = 0;
        float theta = Vector3.Dot(desiredOrientation.normalized, droneOrientation.normalized);
        
        float angle = Mathf.Abs(Vector3.Angle(desiredOrientation, droneOrientation));
        if (parcoursState.currentWaypoint != currentWaypoint)
        {
            currentWaypoint = parcoursState.currentWaypoint;
            previousTheta = theta; // reset previous angle when reaching a new way point
            return 0f;
        }
        
        statsRecorder.Add(loggingKey, angle, StatAggregationMethod.Histogram);
        episodeLogger.Add(ToSnakeCase(loggingKey), angle);

        statsRecorder.Add("Orientation/Angle", angle, StatAggregationMethod.Histogram);
        statsRecorder.Add("Goal/Orientation", angle);
        episodeLogger.Add("goal/orientation", angle);
        episodeLogger.Add("metric/orientation", angle);

        var angleReward = theta - previousTheta;

        previousTheta = theta;

        return angleReward;
    }
}
