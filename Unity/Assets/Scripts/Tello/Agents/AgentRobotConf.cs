using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System;
using Vector3 = UnityEngine.Vector3;
using Quaternion = UnityEngine.Quaternion;
using System.Collections.Generic;
using System.Numerics;

public class AgentRobotConf : RLAgenticController
{
    private ActionBuffers lastActions = new ActionBuffers(new float[4]{0, 0, 0, 0}, new int[4]{0, 0, 0, 0});
    // whether any CBF modified the controller state during the last action
    private bool lastCBFTriggered = false;
    // Per-agent control whether to enable CBF enforcement
    public bool enableCBFs = true;
    public bool enableResetOnFailure = true;

    // Local handler instance owned by this agent
    private CBFHandler cbfHandler;
    private const int TimeoutDiagnosticsWindow = 100;
    private readonly Queue<float> progressHistory = new Queue<float>();
    private readonly Queue<float> distanceToWaypointHistory = new Queue<float>();
    private readonly Queue<float> speedHistory = new Queue<float>();
    private float speedHistorySum = 0f;

    protected override void ResetInternalData()
    {
        // Instantiate local CBF handler and wire loggers
        cbfHandler = new CBFHandler();
        cbfHandler.SetLoggers(statsRecorder, episodeLogger);

        overallReward = new SummedReward("Overall", 
            new HitWaypointReward(1.0f),
            new DistanceReward(1.1f),
            new OrientationReward(4.0f),
            new MaxSpeedReward(0.5f),
            new InCorridorReward(0.03f),
            new ActionChangeReward(0.5f)
        );
        overallReward.SetLoggers(statsRecorder, episodeLogger);

        lastActions = new ActionBuffers(new float[4] { 0, 0, 0, 0 }, new int[4] { 0, 0, 0, 0 });

        lastCBFTriggered = false;
        progressHistory.Clear();
        distanceToWaypointHistory.Clear();
        speedHistory.Clear();
        speedHistorySum = 0f;

        SetIncludeRotation(true);
        SetIncludeSideMotion(true);
        SetUseContinuousActions(true);

        //episodeLogger.loggingEnabled = false;
    }
    
    public override void CollectObservations(VectorSensor sensor)
    {
        if (parcours.waypointType != Waypoint.Type.COMPLEX)
        {
            Debug.LogWarning("Parcours is of wrong type");
            return;
        }

        episodeLogger.Add("time", Time.time);
        ComplexWaypoint waypoint = (ComplexWaypoint) GetCurrentObservedWaypoint();
        var list = parcours.GetWaypointList();
        int waypointIndex = list.IndexOf(waypoint);
        episodeLogger.Add("waypoint/index", waypointIndex);
        if (waypoint)
        {
            episodeLogger.Add("waypoint/position_x", waypoint.transform.position.x);
            episodeLogger.Add("waypoint/position_y", waypoint.transform.position.y);
            episodeLogger.Add("waypoint/position_z", waypoint.transform.position.z);
            episodeLogger.Add("waypoint/orientation_x", waypoint.transform.rotation.x);
            episodeLogger.Add("waypoint/orientation_y", waypoint.transform.rotation.y);
            episodeLogger.Add("waypoint/orientation_z", waypoint.transform.rotation.z);
            episodeLogger.Add("waypoint/orientation_w", waypoint.transform.rotation.w);
            episodeLogger.Add("waypoint/flight_radius", waypoint.flightRadius);
            episodeLogger.Add("waypoint/max_speed", waypoint.maxSpeed);
            episodeLogger.Add("waypoint/drone_orientation_type", (int) waypoint.droneOrientationType);
            episodeLogger.Add("waypoint/drone_orientation_x", waypoint.droneOrientation.x);
            episodeLogger.Add("waypoint/drone_orientation_y", waypoint.droneOrientation.y);
            episodeLogger.Add("waypoint/drone_orientation_z", waypoint.droneOrientation.z);
        }

        DroneState droneState = GetObservedDroneState();
        if (waypoint)
        {
            // the vector from the drone to the next way point in global space
            Vector3 globalDroneToWayPoint = waypoint.transform.position - droneState.position;
            // transform it to local space (relative to the drone orientation)
            Vector3 droneToWayPoint = Quaternion.Inverse(droneState.rotation) * globalDroneToWayPoint;
            try {
                sensor.AddObservation(droneToWayPoint);
                sensor.AddObservation(waypoint.flightRadius);
                sensor.AddObservation(waypoint.maxSpeed);
            } catch (Exception e)
            {
                Debug.LogError($"Error adding observation: {e.Message}, waypoint: {waypoint.transform.position}, drone: {droneState.position}, rotation: {droneState.rotation}");
                sensor.AddObservation(new float[5]);
            }
            switch (waypoint.droneOrientationType)
            {
                case DroneOrientationType.STATIC:
                    Vector3 relativeDirection = Quaternion.Inverse(droneState.rotation) * waypoint.droneOrientation;
                    sensor.AddObservation(relativeDirection);
                    break;
                case DroneOrientationType.TOWARDS_POINT:
                    Vector3 globalDroneToPoint = waypoint.droneOrientation - droneState.position;
                    Vector3 relativeDroneToPoint = Quaternion.Inverse(droneState.rotation) * globalDroneToPoint;
                    sensor.AddObservation(relativeDroneToPoint);
                    break;
                case DroneOrientationType.TOWARDS_WAYPOINT:
                    Vector3 globalDroneToWaypoint = waypoint.transform.position - droneState.position;
                    Vector3 relativeDroneToWaypoint = Quaternion.Inverse(droneState.rotation) * globalDroneToWaypoint;
                    sensor.AddObservation(relativeDroneToWaypoint);
                    break;
            }
        }
        else
        {
            sensor.AddObservation(new float[8]);
        }

        Vector3 previousWaypoint = GetPreviousObservedWaypoint() == null ? parcours.GetStartPosition() : GetPreviousObservedWaypoint().transform.position;
        var aToB = waypoint.transform.position - previousWaypoint;
        var dronePosition = GetObservedDroneState().position;
        var aToDrone = dronePosition - previousWaypoint;
        var angle = Vector3.Angle(aToB, aToDrone) * Mathf.Deg2Rad;
        sensor.AddObservation((float)Math.Sin(angle) * aToDrone.magnitude);

        Vector3 linearVelocity = droneState.linearVelocity;
        Vector3 localVelocity = Quaternion.AngleAxis(90, Vector3.up) * Quaternion.Inverse(droneState.rotation) * linearVelocity;
        sensor.AddObservation(localVelocity);
        float angularVelocity = droneState.angularVelocity;
        sensor.AddObservation(angularVelocity);

        statsRecorder.Add("Drone/LinearVelocity", linearVelocity.magnitude);
        statsRecorder.Add("Drone/LinearVelocityHist", Mathf.Min(linearVelocity.magnitude, 3.0f), StatAggregationMethod.Histogram);
        statsRecorder.Add("Drone/AngularVelocity", angularVelocity);
        statsRecorder.Add("Drone/AbsoluteAngularVelocity", Mathf.Abs(angularVelocity));
        statsRecorder.Add("Drone/ControlForce", Mathf.Max(new float[4]{Mathf.Abs(ActionToInputContinuous(lastActions.ContinuousActions[0])),
                                                                       Mathf.Abs(ActionToInputContinuous(lastActions.ContinuousActions[1])),
                                                                       Mathf.Abs(ActionToInputContinuous(lastActions.ContinuousActions[2])),
                                                                       Mathf.Abs(ActionToInputContinuous(lastActions.ContinuousActions[3]))}));
        statsRecorder.Add("Drone/DirectionForward", ActionToInputContinuous(lastActions.ContinuousActions[0]));
        statsRecorder.Add("Drone/AmplitudeForward", Mathf.Abs(ActionToInputContinuous(lastActions.ContinuousActions[0])));
        statsRecorder.Add("Drone/DirectionSide", ActionToInputContinuous(lastActions.ContinuousActions[1]));
        statsRecorder.Add("Drone/AmplitudeSide", Mathf.Abs(ActionToInputContinuous(lastActions.ContinuousActions[1])));
        statsRecorder.Add("Drone/DirectionUpward", ActionToInputContinuous(lastActions.ContinuousActions[2]));
        statsRecorder.Add("Drone/AmplitudeUpward", Mathf.Abs(ActionToInputContinuous(lastActions.ContinuousActions[2])));
        statsRecorder.Add("Drone/DirectionRotation", ActionToInputContinuous(lastActions.ContinuousActions[3]));
        statsRecorder.Add("Drone/AmplitudeRotation", Mathf.Abs(ActionToInputContinuous(lastActions.ContinuousActions[3])));

        episodeLogger.Add("drone/observed/position_x", droneState.position.x);
        episodeLogger.Add("drone/observed/position_y", droneState.position.y);
        episodeLogger.Add("drone/observed/position_z", droneState.position.z);
        episodeLogger.Add("drone/observed/orientation_x", droneState.rotation.x);
        episodeLogger.Add("drone/observed/orientation_y", droneState.rotation.y);
        episodeLogger.Add("drone/observed/orientation_z", droneState.rotation.z);
        episodeLogger.Add("drone/observed/orientation_w", droneState.rotation.w);
        episodeLogger.Add("drone/observed/linear_velocity", droneState.linearVelocity.magnitude);
        episodeLogger.Add("drone/observed/linear_velocity_x", droneState.linearVelocity.x);
        episodeLogger.Add("drone/observed/linear_velocity_y", droneState.linearVelocity.y);
        episodeLogger.Add("drone/observed/linear_velocity_z", droneState.linearVelocity.z);
        episodeLogger.Add("drone/observed/angular_velocity", droneState.angularVelocity);

        sensor.AddObservation(lastActions.ContinuousActions[0]);
        sensor.AddObservation(lastActions.ContinuousActions[1]);
        sensor.AddObservation(lastActions.ContinuousActions[2]);
        sensor.AddObservation(lastActions.ContinuousActions[3]);
    }

    protected override bool ModifyControllerState(ref ControllerState controllerState)
    {
        ParcoursState parcoursState = new ParcoursState(
            (ComplexWaypoint) GetCurrentObservedWaypoint(),
            (ComplexWaypoint) GetPreviousObservedWaypoint(),
            parcours.GetStartPosition(),
            false,
            StepCount,
            lastWaypointHit,
            lastCBFTriggered
        );
        DroneState droneState = GetObservedDroneState();
        bool modified = false;

        bool cbfsActive = Academy.Instance.EnvironmentParameters.GetWithDefault("cbfs_active", enableCBFs ? 1.0f : 0.0f) > 0.5f;
        if (enableCBFs && cbfsActive)
        {
            modified = cbfHandler.ApplyCBFs(ref controllerState, droneState, parcoursState);
        }
        lastCBFTriggered = modified;
        return modified;
    }

    protected override bool HasFailedTarget(bool hitWaypoint)
    {

        if ((StepCount - lastWaypointHit) <= 20)
        {
            return false;
        }

        Waypoint waypoint = hitWaypoint ? GetPreviousRewardWaypoint() : GetCurrentRewardWaypoint();
        if (!waypoint) return false;

        bool isNotInsideCorridor = !IsDroneInsideCorridor((ComplexWaypoint)waypoint);
        statsRecorder.Add("Failure/DroneNotInsideCorridor", isNotInsideCorridor ? 1 : 0);
        bool isAboveMaxAllowedSpeed = !IsUnderMaxAllowedSpeed((ComplexWaypoint)waypoint);
        statsRecorder.Add("Failure/DroneAboveMaxAllowedSpeed", isAboveMaxAllowedSpeed ? 1 : 0);

        if (isAboveMaxAllowedSpeed)
        {
            statsRecorder.Add("Waypoint/MaxSpeedFailureRate", 1);
            statsRecorder.Add("Waypoint/CorridorFailureRate", 0);
            statsRecorder.Add("Waypoint/FailureParcoursType", parcours.parcoursType == ParcoursGenerator.ParcoursType.RANDOM ? 1 : 0);

            episodeLogger.Add("episode/failure_max_speed", 1);
        }
        if (isNotInsideCorridor)
        {
            statsRecorder.Add("Waypoint/MaxSpeedFailureRate", 0);
            statsRecorder.Add("Waypoint/CorridorFailureRate", 1);
            statsRecorder.Add("Waypoint/FailureParcoursType", parcours.parcoursType == ParcoursGenerator.ParcoursType.RANDOM ? 1 : 0);

            episodeLogger.Add("episode/failure_corridor", 1);
        }

        if (inference) return false;
        if (!enableResetOnFailure) return false;

        return isNotInsideCorridor || isAboveMaxAllowedSpeed;
    }

    private bool IsDroneInsideCorridor(ComplexWaypoint waypoint)
    {
        Vector3 previousWaypoint = GetPreviousRewardWaypoint() == null ? parcours.GetStartPosition() : GetPreviousRewardWaypoint().transform.position;
        var aToB = waypoint.transform.position - previousWaypoint;
        var dronePosition = GetRewardDroneState().position;
        var aToDrone = dronePosition - previousWaypoint;
        var angle = Vector3.Angle(aToB, aToDrone) * Mathf.Deg2Rad;

        episodeLogger.Add("metric/cross_track_distance", (float)Math.Sin(angle) * aToDrone.magnitude);

        return (float)Math.Sin(angle) * aToDrone.magnitude < (waypoint.flightRadius /*+ drone.GetDroneSize()/2*/);
    }

    private bool IsUnderMaxAllowedSpeed(ComplexWaypoint waypoint)
    {
        var magnitude = GetRewardDroneState().linearVelocity.magnitude;

        episodeLogger.Add("metric/speed", magnitude);
        return magnitude <= waypoint.maxSpeed;
    }

    private void logProgress()
    {
        // Get progress on current segment
        Vector3 previousWaypoint = GetPreviousRewardWaypoint() == null ? parcours.GetStartPosition() : GetPreviousRewardWaypoint().transform.position;
        var aToB = GetCurrentRewardWaypoint().transform.position - previousWaypoint;
        var dronePosition = GetRewardDroneState().position;
        var aToDrone = dronePosition - previousWaypoint;
        float progress = Vector3.Dot(aToDrone, aToB.normalized) / aToB.magnitude;
        episodeLogger.Add("metric/progress", progress);
    }

    private float GetCurrentProgressValue()
    {
        Waypoint currentRewardWaypoint = GetCurrentRewardWaypoint();
        if (currentRewardWaypoint == null)
            return 0f;

        Vector3 previousWaypoint = GetPreviousRewardWaypoint() == null ? parcours.GetStartPosition() : GetPreviousRewardWaypoint().transform.position;
        Vector3 segment = currentRewardWaypoint.transform.position - previousWaypoint;
        float segmentLength = segment.magnitude;
        if (segmentLength < 1e-5f)
            return 0f;

        Vector3 dronePosition = GetRewardDroneState().position;
        Vector3 aToDrone = dronePosition - previousWaypoint;
        return Vector3.Dot(aToDrone, segment.normalized) / segmentLength;
    }

    private float GetDistanceToCurrentWaypointValue()
    {
        Waypoint currentRewardWaypoint = GetCurrentRewardWaypoint();
        if (currentRewardWaypoint == null)
            return 0f;

        return Vector3.Distance(GetRewardDroneState().position, currentRewardWaypoint.transform.position);
    }

    private void EnqueueHistoryValue(Queue<float> history, float value)
    {
        history.Enqueue(value);
        if (history.Count > TimeoutDiagnosticsWindow)
        {
            history.Dequeue();
        }
    }

    private void EnqueueSpeedHistoryValue(float speed)
    {
        speedHistory.Enqueue(speed);
        speedHistorySum += speed;
        if (speedHistory.Count > TimeoutDiagnosticsWindow)
        {
            speedHistorySum -= speedHistory.Dequeue();
        }
    }

    private void UpdateTimeoutDiagnostics(DroneState droneState)
    {
        EnqueueHistoryValue(progressHistory, GetCurrentProgressValue());
        EnqueueHistoryValue(distanceToWaypointHistory, GetDistanceToCurrentWaypointValue());
        EnqueueSpeedHistoryValue(droneState.linearVelocity.magnitude);
    }

    private void LogTimeoutDiagnostics()
    {
        float distanceToWaypoint = GetDistanceToCurrentWaypointValue();
        float currentProgress = GetCurrentProgressValue();
        float progressLast100 = progressHistory.Count > 0 ? currentProgress - progressHistory.Peek() : 0f;
        float distanceDeltaLast100 = distanceToWaypointHistory.Count > 0 ? distanceToWaypoint - distanceToWaypointHistory.Peek() : 0f;
        float meanSpeedLast100 = speedHistory.Count > 0 ? speedHistorySum / speedHistory.Count : 0f;

        statsRecorder.Add("Timeout/Count", 1, StatAggregationMethod.Sum);
        statsRecorder.Add("Timeout/WaypointIndex", parcours.GetCurrentWayPointIndex(), StatAggregationMethod.Histogram);
        statsRecorder.Add("Timeout/DistanceToWaypoint", distanceToWaypoint, StatAggregationMethod.Histogram);
        statsRecorder.Add("Timeout/ProgressLast100", progressLast100, StatAggregationMethod.Histogram);
        statsRecorder.Add("Timeout/DistanceDeltaLast100", distanceDeltaLast100, StatAggregationMethod.Histogram);
        statsRecorder.Add("Timeout/MeanSpeedLast100", meanSpeedLast100, StatAggregationMethod.Histogram);
    }

    protected override void CalculateReward(ActionBuffers actions, bool hitWaypoint)
    {
        ParcoursState parcoursState = new ParcoursState(
            (ComplexWaypoint) GetCurrentRewardWaypoint(),
            (ComplexWaypoint) GetPreviousRewardWaypoint(),
            parcours.GetStartPosition(),
            hitWaypoint,
            StepCount,
            lastWaypointHit,
            lastCBFTriggered
        );
        DroneState droneState = GetRewardDroneState();
        float overallRewardValue = overallReward.GetReward(this, actions, droneState, parcoursState);
        UpdateTimeoutDiagnostics(droneState);

        // give rewards every step
        SetReward(overallRewardValue);

        episodeLogger.Add("drone/position_x", drone.transform.position.x);
        episodeLogger.Add("drone/position_y", drone.transform.position.y);
        episodeLogger.Add("drone/position_z", drone.transform.position.z);
        episodeLogger.Add("drone/orientation_x", drone.transform.rotation.x);
        episodeLogger.Add("drone/orientation_y", drone.transform.rotation.y);
        episodeLogger.Add("drone/orientation_z", drone.transform.rotation.z);
        episodeLogger.Add("drone/orientation_w", drone.transform.rotation.w);
        episodeLogger.Add("drone/linear_velocity", drone.GetLinearVelocity().magnitude);
        episodeLogger.Add("drone/linear_velocity_x", drone.GetLinearVelocity().x);
        episodeLogger.Add("drone/linear_velocity_y", drone.GetLinearVelocity().y);
        episodeLogger.Add("drone/linear_velocity_z", drone.GetLinearVelocity().z);
        episodeLogger.Add("drone/angular_velocity", drone.GetAngularVelocity());

        episodeLogger.Add("actions/forward", ActionToInputContinuous(lastActions.ContinuousActions[0]));
        episodeLogger.Add("actions/side", ActionToInputContinuous(lastActions.ContinuousActions[1]));
        episodeLogger.Add("actions/upward", ActionToInputContinuous(lastActions.ContinuousActions[2]));
        episodeLogger.Add("actions/rotation", ActionToInputContinuous(lastActions.ContinuousActions[3]));

        logProgress();

        lastActions = actions;
    }

    protected override void OnParcoursFinished(ActionBuffers actions)
    {
        //AddReward(20.0f);
        episodeLogger.Add("reward/finished", 20.0f);
    }
    
    protected override void OnParcoursTimeout(ActionBuffers actions)
    {
        AddReward(-10.0f);
        LogTimeoutDiagnostics();
        episodeLogger.Add("reward/timeout", -10.0f);
    }

    protected override void OnParcoursFailed(ActionBuffers actions)
    {
        statsRecorder.Add("Waypoint/Failed", 1, StatAggregationMethod.Sum);
        statsRecorder.Add("Waypoint/FailedAtWaypoint", parcours.GetCurrentWayPointIndex(), StatAggregationMethod.Histogram);
        episodeLogger.Add("reward/failure", -10.0f);
    }

    public override float InputToAction(float input)
    {
        return Mathf.Clamp(input + 1.0f, 0.0f, 2.0f);
    }
}
