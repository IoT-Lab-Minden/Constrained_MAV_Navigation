using UnityEngine;

[CreateAssetMenu(menuName = "Drone/SimulationModel/Kinematic")]
public class KinematicSimulationModel : SimulationModel
{
    public KinematicDroneParameters parameters;

    private Vector3 velocity = Vector3.zero;
    private float angularVelocity = 0;
    private float yaw = 0;

    public override SimDroneParameters GetDroneParameters()
    {
        return parameters;
    }

    public override Vector3 GetLinearVelocity()
    {
        return velocity;
    }

    public override float GetAngularVelocty()
    {
        return angularVelocity;
    }

    public override void ResetSimulationModel(Transform drone)
    {
        velocity = Vector3.zero;
        angularVelocity = 0;
        yaw = drone.transform.rotation.eulerAngles.y;
        parameters.InitParameters();
    }

    public override void InitializePredictionState(Transform drone, Vector3 linearVelocity, float angularVelocity)
    {
        velocity = linearVelocity;
        this.angularVelocity = angularVelocity;
        yaw = drone.transform.rotation.eulerAngles.y;
    }

    public override void UpdateOrientation(ControllerState controllerState, Transform drone)
    {
        float dt = Time.fixedDeltaTime;

        float currentMaxAngularVelocity = parameters.GetMaxAngularVelocity() * Mathf.Abs(controllerState.rx);

        if (velocity.magnitude < currentMaxAngularVelocity)
        {
            angularVelocity += controllerState.rx * parameters.GetAngularAcceleration() * dt;
        }
        
        // Yaw drag
        angularVelocity -= angularVelocity * parameters.GetAngularDragCoefficient() * dt;

        // Cap of angular velocity
        angularVelocity = Mathf.Clamp(angularVelocity, -parameters.GetMaxAngularVelocity(), parameters.GetMaxAngularVelocity());

        yaw += angularVelocity * Mathf.Rad2Deg * dt;

        drone.transform.rotation = Quaternion.Euler(0f, yaw, 0f);
    }

    public override void UpdatePosition(ControllerState controllerState, Transform drone)
    {
        float dt = Time.fixedDeltaTime;

        Vector3 input = new Vector3(controllerState.ly, controllerState.ry, -controllerState.lx);

        float currentMaxLinearVelocity = parameters.GetMaxLinearVelocity() * Mathf.Abs(input.magnitude);

        if (velocity.magnitude < currentMaxLinearVelocity)
        {
            velocity += drone.transform.rotation * (input * parameters.GetLinearAcceleration() * dt);
        }

        // linear drag
        velocity -= parameters.GetLinearDragCoefficient() /** velocity.magnitude*/ * velocity * dt;

        // Cap of linear velocity
        velocity = Vector3.ClampMagnitude(velocity, parameters.GetMaxLinearVelocity());

        drone.transform.position += velocity * dt;
    }
}