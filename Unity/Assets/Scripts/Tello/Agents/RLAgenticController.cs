using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using System;
using System.Collections.Generic;

public abstract class RLAgenticController : Agent
{
    public GameObject dronePrefab;
    protected Reward overallReward;

    protected EvalLogger logger;

    protected Drone drone;

    public Parcours parcours;
    protected int episodeCounter;
    protected float startTime;

    public bool inference = false;
    public bool heuristic = false;

    public bool useDelayedRewardState = false;
    public bool useDelayedObservationState = true;
    private Waypoint currentWaypoint;
    private Waypoint previousWaypoint;
    private Waypoint nextWaypoint;
    private Queue<long> pendingHitQueue = new Queue<long>();

    private bool includeRotation = false;
    private bool includeSideMotion = true;
    private bool useContinuousActions = false;

    protected StatsRecorder statsRecorder;
    public EpisodeLogger episodeLogger;

    private bool parcoursInitialized = false;
    private ControllerState lastControllerState;

    new void Awake()
    {
        statsRecorder = Academy.Instance.StatsRecorder;

        drone = Instantiate(dronePrefab, this.transform).GetComponent<Drone>();
        drone.StartConnecting();

        logger = new EvalLogger();
        logger.enabled = false;
        episodeLogger = new EpisodeLogger();

        lastControllerState = new ControllerState();
        startTime = Time.time;
    }

    void Start()
    {
        // We need to place this here, as the parcour is unknown at awake time
        InitParcours();
    }

    private bool InitParcours()
    {
        if (!parcoursInitialized && parcours != null)
        {
            parcours.agent = drone.gameObject;
            parcours.SetOnWaypointReachedCallback(OnWaypointReachedCallback);
            parcours.SetOnParcoursFinishedCallback(OnParcoursFinishedCallback);
            parcours.SetOnWaypointHitCallback(OnWaypointHitCallbackInternal);
            // initialize local waypoint cache
            currentWaypoint = parcours.GetCurrentWayPoint();
            previousWaypoint = parcours.GetPreviousWayPoint();
            nextWaypoint = parcours.GetNextWaypoint();
            parcoursInitialized = true;
        }

        return parcoursInitialized;
    }

    void OnApplicationQuit()
    {
        drone.StopConnecting();
        logger.Close();
    }

    public Drone GetDrone()
    {
        return drone;
    }
    
    protected static float CrossTrackError(Vector3 wayPointA, Vector3 wayPointB, Vector3 dronePosition)
    {
        var aToB = wayPointB - wayPointA;
        var aToDrone = dronePosition - wayPointA;
        var angle = Vector3.Angle(aToB, aToDrone) * Mathf.Deg2Rad;

        return (float)Math.Sin(angle) * aToDrone.magnitude;
    }

    protected DroneState GetRewardDroneState()
    {
        if (useDelayedRewardState)
        {
            return drone.GetDroneState();
        }
        else
        {
            return new DroneState(
                drone.flightState,
                drone.transform.position,
                drone.transform.rotation,
                drone.GetLinearVelocity(),
                drone.GetAngularVelocity(),
                drone.GetDroneSize()
            );
        }
    }

    protected DroneState GetObservedDroneState()
    {
        if (useDelayedObservationState)
        {
            return drone.GetDroneState();
        }
        else
        {
            return new DroneState(
                drone.flightState,
                drone.transform.position,
                drone.transform.rotation,
                drone.GetLinearVelocity(),
                drone.GetAngularVelocity(),
                drone.GetDroneSize()
            );
        }
    }

    private void ResetDroneAndParcours()
    {
        if (!InitParcours()) return;
        // sets the parameters for the parcours
        parcours.sizeModifier = Academy.Instance.EnvironmentParameters.GetWithDefault("size_modifier", 1.5f);

        // reset the position and velocity of the drone
        var randomPositionOffset = new Vector3((float)(UnityEngine.Random.value * parcours.sizeModifier * 2) - parcours.sizeModifier,
                                            (float)(UnityEngine.Random.value * parcours.sizeModifier * 2) - parcours.sizeModifier,
                                            (float)(UnityEngine.Random.value * parcours.sizeModifier * 2) - parcours.sizeModifier);
        Vector3 startPosition = parcours.transform.position + randomPositionOffset + new Vector3(0, 0.1f, 0);
        drone.transform.position = startPosition;
        drone.transform.rotation = Quaternion.Euler(0, (float)UnityEngine.Random.value * 360.0f, 0);
        drone.GetComponent<Rigidbody>().linearVelocity = Vector3.zero;
        drone.GetComponent<Rigidbody>().angularVelocity = Vector3.zero;
        drone.GetComponent<Rigidbody>().Sleep();
        drone.SetControllerState(new ControllerState());
        drone.ResetDrone();

        // reset the environment
        parcours.ResetParcours(startPosition);
        nextWaypoint = parcours.GetNextWaypoint();
        currentWaypoint = parcours.GetCurrentWayPoint();
        previousWaypoint = parcours.GetPreviousWayPoint();

        lastControllerState = new ControllerState();
        startTime = Time.time;
    }

    public override void OnEpisodeBegin()
    {
        episodeCounter++;
        if (!inference)
        {
            if (episodeCounter % 50 == 0)
                Debug.Log("Episode: " + episodeCounter);

            ResetDroneAndParcours();
        }
        else
        {
            if (episodeCounter > 1)
            {
                this.enabled = false;
                episodeCounter = 0;
                return;
            }
            if (parcoursInitialized)
                parcours.ResetParcours(drone.transform.position);
        }

        ResetInternalData();
        pendingHitQueue.Clear();

        // reset attributes used for the reward
        lastWaypointHit = StepCount;
    }

    protected abstract void ResetInternalData();

    protected int lastWaypointHit;
    
    public virtual float ActionToInputDiscrete(float action) {
        return Mathf.Clamp(action - 1.0f, -1.0f, 1.0f);
    }
    
    public virtual float ActionToInputContinuous(float action) {
        return action; // Should already be in the range of -1 to 1
    }

    public abstract float InputToAction(float input);

    // controls the top speed of the drone, simulation typically runs on 1.0, while the real drone achieved the best result on 0.3
    public float speedMultiplier = 1.0f;

    protected virtual bool ModifyControllerState(ref ControllerState controllerState)
    {
        return false;
    }

    private bool hitWaypoint = false, parcoursFinished = false;

    protected Waypoint GetCurrentRewardWaypoint()
    {
        if (useDelayedRewardState) return currentWaypoint;
        if (hitWaypoint) return parcours.GetPreviousWayPoint();
        return parcours.GetCurrentWayPoint();
    }

    protected Waypoint GetNextRewardWaypoint()
    {
        if (useDelayedRewardState) return nextWaypoint;
        if (hitWaypoint) return parcours.GetCurrentWayPoint();
        return parcours.GetNextWaypoint();
    }

    protected Waypoint GetPreviousRewardWaypoint()
    {
        if (useDelayedRewardState) return previousWaypoint;
        if (hitWaypoint) return parcours.GetSecondPreviousWaypoint();
        return parcours.GetPreviousWayPoint();
    }

    protected Waypoint GetCurrentObservedWaypoint()
    {
        if (useDelayedObservationState) return currentWaypoint;
        if (hitWaypoint) return parcours.GetPreviousWayPoint();
        return parcours.GetCurrentWayPoint();
    }

    protected Waypoint GetNextObservedWaypoint()
    {
        if (useDelayedObservationState) return nextWaypoint;
        if (hitWaypoint) return parcours.GetCurrentWayPoint();
        return parcours.GetNextWaypoint();
    }

    protected Waypoint GetPreviousObservedWaypoint()
    {
        if (useDelayedObservationState) return previousWaypoint;
        if (hitWaypoint) return parcours.GetSecondPreviousWaypoint();
        return parcours.GetPreviousWayPoint();
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        if (heuristic) return;
        episodeLogger.Add("trajectory_time", Time.time - startTime);

        float pitch, roll, elevation, yaw;
        // convert actions to inputs
        if (useContinuousActions)
        {
            pitch = ActionToInputContinuous(actions.ContinuousActions[0]);
            roll = ActionToInputContinuous(actions.ContinuousActions[1]);
            elevation = ActionToInputContinuous(actions.ContinuousActions[2]);
            yaw = ActionToInputContinuous(actions.ContinuousActions[3]);
        }
        else
        {
            pitch = ActionToInputDiscrete(actions.DiscreteActions[0]);
            roll = ActionToInputDiscrete(actions.DiscreteActions[1]);
            elevation = ActionToInputDiscrete(actions.DiscreteActions[2]);
            yaw = ActionToInputDiscrete(actions.DiscreteActions[3]);
        }

        // automatically take off, if not in the air
        if (drone.GetDroneState().flightState == FlightState.LANDED)
        {
            drone.TakeOff();
        }

        // set the new input for the drone
        ControllerState newState = new(
            includeSideMotion ? roll * speedMultiplier : 0.0f,
            pitch * speedMultiplier,
            includeRotation ? yaw * speedMultiplier : 0.0f,
            elevation * speedMultiplier
        );

        float alpha = 1.0f;
        ControllerState finalState = new(
            alpha * newState.lx + (1 - alpha) * lastControllerState.lx,
            alpha * newState.ly + (1 - alpha) * lastControllerState.ly,
            alpha * newState.rx + (1 - alpha) * lastControllerState.rx,
            alpha * newState.ry + (1 - alpha) * lastControllerState.ry
        );
        lastControllerState = finalState;

        ModifyControllerState(ref finalState);
        drone.SetControllerState(finalState);

        CalculateReward(actions, hitWaypoint);

        if (hitWaypoint)
        {
            lastWaypointHit = StepCount;
        }

        if (parcoursFinished)
        {
            statsRecorder.Add("Waypoint/ParcoursFinished", 1, StatAggregationMethod.Sum);
            statsRecorder.Add("Waypoint/ParcoursSuccessRate", 1);
            statsRecorder.Add("Waypoint/TimeoutRate", 0);
            statsRecorder.Add("Waypoint/MaxSpeedFailureRate", 0);
            statsRecorder.Add("Waypoint/CorridorFailureRate", 0);
            statsRecorder.Add("Waypoint/FinishedEpisodeLength", StepCount);
            episodeLogger.Add("episode/parcours_finished", 1);
            episodeLogger.NextStep();
            episodeLogger.EndEpisode();
            OnParcoursFinished(actions);
            EndEpisode();
            drone.SetControllerState(new ControllerState(0, 0, 0, 0));
            //Debug.Log("Parcours finished");
            parcoursFinished = false;
        }
        else if (StepCount > 1000 * (parcours.GetCurrentWayPointIndex() + 1) && !inference && !heuristic)
        {
            statsRecorder.Add("Waypoint/TimeoutRate", 1);
            statsRecorder.Add("Waypoint/MaxSpeedFailureRate", 0);
            statsRecorder.Add("Waypoint/CorridorFailureRate", 0);
            statsRecorder.Add("Waypoint/ParcoursSuccessRate", 0);
            episodeLogger.Add("episode/timeout", 1);
            episodeLogger.NextStep();
            episodeLogger.EndEpisode();
            OnParcoursTimeout(actions);
            //Debug.Log("Parcours timeout");
            EpisodeInterrupted();
        }
        else if (HasFailedTarget(hitWaypoint))
        {
            OnParcoursFailed(actions);
            statsRecorder.Add("Waypoint/TimeoutRate", 0);
            statsRecorder.Add("Waypoint/ParcoursSuccessRate", 0);
            episodeLogger.Add("episode/failure", 1);
            episodeLogger.NextStep();
            episodeLogger.EndEpisode();
            //Debug.Log("Parcours failed");
            EpisodeInterrupted();
        }

        if (hitWaypoint && overallReward != null)
        {
            overallReward.ResetRewardOnWaypointReached();
        }

        hitWaypoint = false;
        if (pendingHitQueue.Count > 0)
        {
            var currentReal = drone.GetDroneState();

            while (pendingHitQueue.Count > 0)
            {
                var entry = pendingHitQueue.Peek();
                long seq = entry;

                if (currentReal.sequenceId >= seq)
                {
                    previousWaypoint = currentWaypoint;
                    currentWaypoint = parcours.GetCurrentWayPoint();
                    nextWaypoint = parcours.GetNextWaypoint();
                    if (useDelayedRewardState) {
                        hitWaypoint = true;
                    }
                    pendingHitQueue.Dequeue();
                    continue;
                }

                // currentReal.sequenceId < seq -> the desired state has not arrived yet
                break;
            }
        }

        episodeLogger.NextStep();
    }

    void OnDestroy()
    {
        episodeLogger.EndEpisode();
    }

    private void OnWaypointHitCallbackInternal(int waypointIndex)
    {
        episodeLogger.Add("waypoint/hit", waypointIndex);
    }

    private void OnWaypointReachedCallback()
    {
        if (parcours.waypointType == Waypoint.Type.COMPLEX)
        {
            ComplexWaypoint waypoint = (ComplexWaypoint)parcours.GetCurrentWayPoint();
            switch (waypoint.droneOrientationType)
            {
                case DroneOrientationType.STATIC:
                    statsRecorder.Add("Waypoint/TypeStatic", 1, StatAggregationMethod.Sum);
                    break;
                case DroneOrientationType.TOWARDS_POINT:
                    statsRecorder.Add("Waypoint/TypeTowardPoint", 1, StatAggregationMethod.Sum);
                    break;
                case DroneOrientationType.TOWARDS_WAYPOINT:
                    statsRecorder.Add("Waypoint/TypeTowardWaypoint", 1, StatAggregationMethod.Sum);
                    break;
            }
            statsRecorder.Add("Waypoint/MaxSpeed", waypoint.maxSpeed);
            statsRecorder.Add("Waypoint/FlightRadius", waypoint.flightRadius);
        }
        
        pendingHitQueue.Enqueue(drone.GetCurrentRealDroneState().sequenceId);
        if (!useDelayedRewardState)
        {
            hitWaypoint = true;
        }
    }

    private void OnParcoursFinishedCallback() {
        parcoursFinished = true;
    }

    protected virtual bool HasFailedTarget(bool hitWaypoint) { return false; }

    protected virtual void OnParcoursFinished(ActionBuffers actions) {}

    protected virtual void OnParcoursTimeout(ActionBuffers actions) {}

    protected virtual void OnParcoursFailed(ActionBuffers actions) {}

    protected abstract void CalculateReward(ActionBuffers actions, bool hitWaypoint);

    public void SetIncludeRotation(bool includeRotation) {
        this.includeRotation = includeRotation;
    }

    public void SetIncludeSideMotion(bool includeSideMotion) {
        this.includeSideMotion = includeSideMotion;
    }

    public void SetUseContinuousActions(bool useContinuousActions) {
        this.useContinuousActions = useContinuousActions;
    }
}
