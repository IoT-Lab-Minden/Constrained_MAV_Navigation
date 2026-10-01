using System;
using System.Collections;
using UnityEngine;


[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(BoxCollider))]
public class SimulationDrone : Drone
{
    [Header("State")]
    public bool connected;
    public float height;
    private DroneState droneState;
    private DroneState currentRealState;
    private long nextSequenceId = 1;

    [Header("Configuration")]
    [SerializeField]
    [Range(1, 20)]
    private int stateUpdateRate = 10; // How many times per second the state is updated
    private float nextStateUpdateTime = 0f;
    
    public Transform heightSensor;

    private SimulationModel simulationModelInstance;

    private ControllerState controllerState;

    private new void Awake()
    {
        base.Awake();
        simulationModelInstance = SimulationModel.Clone(simulationModel);
        connected = false;
        droneType = DroneType.Simulation;
        flightState = FlightState.LANDED;
        GetComponent<Rigidbody>().mass = simulationModelInstance.GetDroneParameters().GetWeight();
        GetComponent<Rigidbody>().useGravity = false;
        controllerState = new ControllerState();
        
        nextStateUpdateTime = Time.fixedTime;
    }

    // Start is called before the first frame update
    void Start()
    {
        // Init current rotation state
        ResetDrone();
    }

    // Update is called once per frame
    void Update()
    {

    }

    void FixedUpdate()
    {
        if (connected)
        {
            // Continuously measure the height
            //MeasureHeight(); // Currently not used

            // Only if flying
            if (InFlyingState())
            {
                // Move drone based on controller state
                HandlePhysics();
            }
            
            // If next state update required
            if (Time.fixedTime >= nextStateUpdateTime)
            {
                // Set next update time
                float period = Mathf.Max(1f / stateUpdateRate, Time.fixedDeltaTime);
                nextStateUpdateTime = Time.fixedTime + period;

                // Enqueue state update with delay
                
                DroneState rawState = new DroneState(
                    flightState,
                    transform.position,
                    transform.rotation,
                    GetLinearVelocity(),
                    GetAngularVelocity(),
                    GetDroneSize(),
                    nextSequenceId++);
                currentRealState = rawState;
                stateDelay.Enqueue(rawState);
            }
        }
    }

    public override void ResetDrone()
    {
        ResetDelays();
        controllerState = new ControllerState();
        simulationModelInstance.ResetSimulationModel(this.transform);
        nextStateUpdateTime = Time.fixedTime;

        DroneState resetState = new DroneState(
            flightState,
            transform.position,
            transform.rotation,
            GetLinearVelocity(),
            GetAngularVelocity(),
            GetDroneSize(),
            nextSequenceId++);
        currentRealState = resetState;
        droneState = resetState;
    }

    private void MeasureHeight()
    {
        RaycastHit hit;
        // Since we take the rotation of the sensor into consideration, we include the drone rotation
        if (Physics.Raycast(heightSensor.position, transform.TransformDirection(Vector3.down), out hit, simulationModelInstance.GetDroneParameters().GetMaxHeight()))
        {
            height = hit.distance;
        }
        else
        {
            height = float.NaN;
        }
    }

    private bool InFlyingState()
    {
        // Returns true if the drone is in any of the flying states
        return flightState == FlightState.FLYING
            || flightState == FlightState.TAKINGOFF
            || flightState == FlightState.LANDING;
    }

    private void HandlePhysics()
    {
        simulationModelInstance.UpdateOrientation(controllerState, this.transform);
        simulationModelInstance.UpdatePosition(controllerState, this.transform);
    }

    public override float GetAngularVelocity()
    {
        return simulationModelInstance.GetAngularVelocty();
    }

    public override Vector3 GetLinearVelocity()
    {
        return simulationModelInstance.GetLinearVelocity();
    }

    public override bool IsConnected()
    {
        return connected;
    }

    public override void StartConnecting()
    {
        // Call intermediate connection state first. Even though we directly
        // set it connected afterwards
        CallOnConnection(new ConnectionStateEvent(ConnectionState.Connecting));
        connected = true;
        Debug.Log("Connected to simulation drone");
        CallOnConnection(new ConnectionStateEvent(ConnectionState.Connected));
    }

    public override void StopConnecting()
    {
        connected = false;
        Debug.Log("Disconnected from simulation drone");
        CallOnConnection(new ConnectionStateEvent(ConnectionState.Disconnected));
    }

    public override void TakeOff()
    {
        // A drone can only takeoff if it is landed
        if (flightState == FlightState.LANDED)
        {
            Debug.Log("Takeoff started");
            controllerState = new ControllerState(0, 0, 0, 1); // Start takeoff
            flightState = FlightState.TAKINGOFF;
            StartCoroutine(StopTakeoff()); // Stop the takeoff later
        }
    }

    public override void Land()
    {
        // A drone can only land if it is flying
        if (flightState == FlightState.FLYING)
        {
            controllerState = new ControllerState(0, 0, 0, -1); // Start landing
            flightState = FlightState.LANDING;
            StartCoroutine(StopLanding()); // Stop the landing later
        }
    }

    public override void SetControllerState(ControllerState state)
    {
        actionDelay.Enqueue(state);
    }

    public override void SetControllerStateImmediate(ControllerState state)
    {
        // Only allow setting controller state, if the drone is actually flying
        if (flightState == FlightState.FLYING)
            controllerState = state;
    }

    public override void SetDroneState(DroneState state)
    {
        if (state.sequenceId > droneState.sequenceId)
            this.droneState = state;
    }

    public override DroneState GetDroneState()
    {
        return droneState;
    }

    public override DroneState GetCurrentRealDroneState()
    {
        return currentRealState;
    }

    public override Pose GetPose()
    {
        return new Pose(transform.position, transform.rotation);
    }

    public override void StartVideo()
    {
        // TODO: Implement video stream
    }

    IEnumerator StopTakeoff()
    {
        // Wait a bit to let the drone reach a certain height
        yield return new WaitForSeconds(simulationModelInstance.GetDroneParameters().GetTakeoffTime());
        controllerState = new ControllerState(); // Reset controller state
        // Wait a bit more to make it hover in the air first
        yield return new WaitForSeconds(simulationModelInstance.GetDroneParameters().GetTakeoffTime());
        flightState = FlightState.FLYING;
        CallOnTakeoff(EventArgs.Empty);
        //Debug.Log("Takeoff finished");
    }

    IEnumerator StopLanding()
    {
        // Wait a bit to let the drone reach the ground
        // TODO: We could slow down the movement just before hitting the ground
        // to make the landing look smoother
        while (height > 0.04)
        {
            // The time doesn't really matter here that much
            // We just need to wait for the height to be updated
            yield return new WaitForSeconds(0.05f);
        }
        controllerState = new ControllerState(); // Reset controller state
        flightState = FlightState.LANDED;
        CallOnLand(EventArgs.Empty);
    }
}
