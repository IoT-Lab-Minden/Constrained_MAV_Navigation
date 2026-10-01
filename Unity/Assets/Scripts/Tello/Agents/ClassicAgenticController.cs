using System.Collections.Generic;
using UnityEngine;

public abstract class ClassicAgenticController : MonoBehaviour
{
    public GameObject dronePrefab;
    public Parcours parcours;
    protected Drone drone;

    [Header("Settings")]
    [Range(1, 50)] public int actionFrequencyHz = 10;
    [Range(0, 10)] public float afterTakeoffDelay = 3f;
    [Range(0.05f, 1.10f)] public float trajectoryDesiredSpeed = 0.5f;
    
    private bool parcoursInitialized = false;
    public Trajectory cachedGenerator = null;

    private float actionTimer = 0f;
    private float afterTakeoffDelayCounter = 0f;

    protected float time = 0f;

    [Header("Safety")]
    public bool enableCBFs = false;
    private CBFHandler cbfHandler;
    private int i = 0;
    public EpisodeLogger episodeLogger;

    public void Awake() {
        // instantiate CBF handler for this agent
        cbfHandler = new CBFHandler();
        episodeLogger = new EpisodeLogger();

        drone = Instantiate(dronePrefab, this.transform).GetComponent<Drone>();
        drone.StartConnecting();

        InitParcours();
    }

    public void Start()
    {
    }

    public virtual void OnDestroy()
    {
        drone.StopConnecting();           
    }

    public bool InitParcours()
    {
        if (!parcoursInitialized && parcours != null)
        {
            parcours.agent = drone.gameObject;
            parcours.SetOnWaypointReachedCallback(OnWaypointReachedCallback);
            parcours.SetOnParcoursFinishedCallback(OnParcoursFinishedCallback);
            parcours.SetOnWaypointHitCallback(OnWaypointHitCallbackInternal);
            parcoursInitialized = true;
        }

        return parcoursInitialized;
    }

    void OnApplicationQuit()
    {
        drone.StopConnecting();
    }

    public void FixedUpdate()
    {

        if (drone.GetDroneState().flightState == FlightState.LANDED)
        {
            drone.TakeOff();
            afterTakeoffDelayCounter = afterTakeoffDelay;
        }

        if (drone.GetDroneState().flightState != FlightState.FLYING)
        {
            return;
        }

        if (afterTakeoffDelayCounter > 0f)
        {
            afterTakeoffDelayCounter -= Time.fixedDeltaTime;
            actionTimer = 0f;
            return;
        }

        if (cachedGenerator == null) {
            cachedGenerator = new OnlineMinSnapTrajectory(parcours, trajectoryDesiredSpeed);
            // enable debug prints for this generator so corridor enforcement logs are visible
            //OnlineMinSnapTrajectory.DebugPrint = true;
            if (drone != null) cachedGenerator.SetDrone(drone);
            cachedGenerator.GenerateTrajectory();
        }
        
        if (!cachedGenerator.HasSolution())
        {
            return;
        }

        float interval = 1f / actionFrequencyHz;
        actionTimer += Time.fixedDeltaTime;

        if (actionTimer >= interval)
        {
            ComplexWaypoint waypoint = (ComplexWaypoint)parcours.GetCurrentWayPoint();
            if (episodeLogger != null && waypoint != null)
            {
                episodeLogger.Add("time", Time.time);
                episodeLogger.Add("trajectory_time", time);
                episodeLogger.Add("waypoint/index", parcours.GetCurrentWayPointIndex());
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

            ControllerState input = GenerateAction();
            // CBFs not used in the evaluation of the paper for the classical methods
            /*if (enableCBFs && cbfHandler != null)
            {
                DroneState droneState2 = new DroneState(
                    drone.GetDroneState().flightState,
                    drone.transform.position,
                    drone.transform.rotation,
                    drone.GetLinearVelocity(),
                    drone.GetAngularVelocity(),
                    drone.GetDroneSize(),
                    i);
                i++;
                ParcoursState parcoursState = new ParcoursState(
                    (ComplexWaypoint)parcours.GetCurrentWayPoint(),
                    (ComplexWaypoint)parcours.GetPreviousWayPoint(),
                    parcours.GetStartPosition(),
                    false, 0, 0, false);
                cbfHandler.ApplyCBFs(ref input, droneState2, parcoursState);
            }*/

            DroneState droneState = drone.GetDroneState();
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

            episodeLogger.Add("actions/forward", input.ly);
            episodeLogger.Add("actions/side", input.lx);
            episodeLogger.Add("actions/upward", input.ry);
            episodeLogger.Add("actions/rotation", input.rx);

            LogMetrics();
            logProgress();

            drone.SetControllerState(input);

            actionTimer -= interval;

            if (time > cachedGenerator.GetTotalTime()+0.5) {
                episodeLogger.Add("parcours_finished", parcours.GetCurrentWayPointIndex() >= parcours.GetWaypointList().Count-1 ? 1 : 0);
                episodeLogger.NextStep();
                episodeLogger.EndEpisode();
                drone.SetControllerState(new ControllerState(0, 0, 0, 0));
                cachedGenerator = null;
            }
            episodeLogger.NextStep();
        }
    }

    private void LogMetrics()
    {
        ComplexWaypoint currentWaypoint = GetCurrentWaypoint(time);
        if (currentWaypoint == null) return;
        episodeLogger.Add("waypoint_timed/index_return", GetCurrentWaypointIndex(time));
        episodeLogger.Add("waypoint_timed/index", parcours.GetCurrentWayPointIndex());
        episodeLogger.Add("waypoint_timed/position_x", currentWaypoint.transform.position.x);
        episodeLogger.Add("waypoint_timed/position_y", currentWaypoint.transform.position.y);
        episodeLogger.Add("waypoint_timed/position_z", currentWaypoint.transform.position.z);
        episodeLogger.Add("waypoint_timed/orientation_x", currentWaypoint.transform.rotation.x);
        episodeLogger.Add("waypoint_timed/orientation_y", currentWaypoint.transform.rotation.y);
        episodeLogger.Add("waypoint_timed/orientation_z", currentWaypoint.transform.rotation.z);
        episodeLogger.Add("waypoint_timed/orientation_w", currentWaypoint.transform.rotation.w);
        episodeLogger.Add("waypoint_timed/flight_radius", currentWaypoint.flightRadius);
        episodeLogger.Add("waypoint_timed/max_speed", currentWaypoint.maxSpeed);
        episodeLogger.Add("waypoint_timed/drone_orientation_type", (int) currentWaypoint.droneOrientationType);
        episodeLogger.Add("waypoint_timed/drone_orientation_x", currentWaypoint.droneOrientation.x);
        episodeLogger.Add("waypoint_timed/drone_orientation_y", currentWaypoint.droneOrientation.y);
        episodeLogger.Add("waypoint_timed/drone_orientation_z", currentWaypoint.droneOrientation.z);

        // Orientation
        Vector3 droneOrientation = Quaternion.AngleAxis(90, Vector3.up) * drone.transform.forward;
        droneOrientation.y = 0;
        
        Vector3 desiredOrientation = Vector3.zero;
        switch (currentWaypoint.droneOrientationType)
        {
            case DroneOrientationType.TOWARDS_WAYPOINT:
                desiredOrientation = Vector3.Normalize(currentWaypoint.transform.position - drone.transform.position);
                break;
            case DroneOrientationType.TOWARDS_POINT:
                desiredOrientation = Vector3.Normalize(currentWaypoint.droneOrientation - drone.transform.position);
                break;
            case DroneOrientationType.STATIC:
                desiredOrientation = currentWaypoint.droneOrientation;
                break;
        }
        desiredOrientation.y = 0;
        
        float angle = Mathf.Abs(Vector3.Angle(desiredOrientation, droneOrientation));
        episodeLogger.Add("metric/orientation", angle);

        // Speed
        float speed = drone.GetLinearVelocity().magnitude;
        episodeLogger.Add("metric/speed", speed);

        // Cross-track distance
        int previousWaypointIndex = GetCurrentWaypointIndex(time) - 1;
        Vector3 previousWaypoint = GetWaypointPosition(previousWaypointIndex);
        var aToB = currentWaypoint.transform.position - previousWaypoint;
        var aToDrone = drone.transform.position - previousWaypoint;
        var angle2 = Vector3.Angle(aToB, aToDrone) * Mathf.Deg2Rad;
        var crossTrackDistance = Mathf.Sin(angle2) * aToDrone.magnitude;

        if (previousWaypointIndex >1) {
            Vector3 prevPrevWaypoint = GetWaypointPosition(previousWaypointIndex - 1);
            var aToB2 = previousWaypoint - prevPrevWaypoint;
            var aToDrone2 = drone.transform.position - prevPrevWaypoint;
            var angle3 = Vector3.Angle(aToB2, aToDrone2) * Mathf.Deg2Rad;
            var crossTrackDistance2 = Mathf.Sin(angle3) * aToDrone2.magnitude;
            crossTrackDistance = Mathf.Min(Mathf.Abs(crossTrackDistance), Mathf.Abs(crossTrackDistance2));
        }
        episodeLogger.Add("metric/cross_track_distance", crossTrackDistance);
        episodeLogger.Add("metric/prev_wp_x", previousWaypoint.x);
        episodeLogger.Add("metric/prev_wp_y", previousWaypoint.y);
        episodeLogger.Add("metric/prev_wp_z", previousWaypoint.z);
    }

    private void logProgress()
    {
        ComplexWaypoint currentWaypoint = GetCurrentWaypoint(time);
        Vector3 previousWaypoint = GetWaypointPosition(GetCurrentWaypointIndex(time) - 1);
        if (currentWaypoint == null) return;

        var aToB = currentWaypoint.transform.position - previousWaypoint;
        var dronePosition = drone.transform.position;
        var aToDrone = dronePosition - previousWaypoint;
        float progress = Vector3.Dot(aToDrone, aToB.normalized) / aToB.magnitude;
        episodeLogger.Add("metric/progress", progress);
    }

    public Drone GetDrone()
    {
        return drone;
    }

    public ComplexWaypoint GetCurrentWaypoint(float trajectoryTime)
    {
        List<Waypoint> waypoints = parcours.GetWaypointList();
        if (waypoints == null || waypoints.Count == 0) return null;

        int waypointIndex = GetCurrentWaypointIndex(trajectoryTime);
        return waypoints[waypointIndex] as ComplexWaypoint;
    }

    public Vector3 GetWaypointPosition(int index)
    {
        if (index < 0) return parcours.GetStartPosition();
        List<Waypoint> waypoints = parcours.GetWaypointList();
        if (waypoints == null || index >= waypoints.Count) return Vector3.zero;
        return waypoints[index].transform.position;
    }

    public int GetCurrentWaypointIndex(float trajectoryTime)
    {
        if (cachedGenerator == null || parcours == null) return -1;

        List<float> segmentTimes = cachedGenerator.GetSegmentTimes();
        if (segmentTimes == null || segmentTimes.Count < 2) return -1;

        int segmentIndex = 0;
        while (segmentIndex < segmentTimes.Count && trajectoryTime > segmentTimes[segmentIndex])
        {
            segmentIndex++;
        }

        int waypointIndex = Mathf.Clamp(segmentIndex, 0, parcours.GetWaypointList().Count);
        return waypointIndex-1;
    }

    protected abstract ControllerState GenerateAction();

    private void OnWaypointHitCallbackInternal(int waypointIndex)
    {
        episodeLogger.Add("waypoint/hit", waypointIndex);
    }

    protected virtual void OnWaypointReachedCallback()
    {
        episodeLogger.Add("waypoint/reached", parcours.GetCurrentWayPointIndex());
    }

    protected virtual void OnParcoursFinishedCallback()
    {
        // base agent does nothing, override in subclasses
    }
}
