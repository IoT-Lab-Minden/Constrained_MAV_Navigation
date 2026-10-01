using UnityEngine;

public class MaxSpeedCBF : ControlBarrierFunction
{
    public MaxSpeedCBF()
    {
        SetSafetyMargin(0.1f);
    }

    public override bool ApplyCBF(ref ControllerState controllerState, DroneState drone, ParcoursState parcours)
    {
        if (parcours.currentWaypoint == null || parcours.currentWaypoint is not ComplexWaypoint)
        {
            if (statsRecorder != null && episodeLogger != null)
            {
                statsRecorder.Add("CBF/MaxSpeed", 0);
                episodeLogger.Add("cbf/max_speed", 0);
            }
            return false; // MaxSpeedCBF only defined for ComplexWaypoints
        }

        float maxSpeed = parcours.currentWaypoint.maxSpeed;
        Vector3 velocity = drone.linearVelocity;
        float h0 = maxSpeed*(1-safetyMargin) - velocity.magnitude;

        if (h0 < 0.0f)
        {
            Vector3 inputLocal = new Vector3(controllerState.ly, controllerState.ry, -controllerState.lx) / 1.10f;
            Quaternion yawRotation = Quaternion.Euler(0f, drone.rotation.eulerAngles.y, 0f);
            Vector3 inputWorld = yawRotation * inputLocal;
            Vector3 vDir = velocity.normalized;
            float thrustAlongV = Vector3.Dot(inputWorld, vDir);
            if (thrustAlongV > 0f)
            {
                inputWorld -= thrustAlongV * vDir;

                Vector3 thrustLocal = Quaternion.Inverse(yawRotation) * inputWorld;
                controllerState.ly = Mathf.Clamp(thrustLocal.x, -1f, 1f);
                controllerState.ry = Mathf.Clamp(thrustLocal.y, -1f, 1f);
                controllerState.lx = Mathf.Clamp(-thrustLocal.z, -1f, 1f);

                if (statsRecorder != null && episodeLogger != null)
                {
                    statsRecorder.Add("CBF/MaxSpeed", 1);
                    episodeLogger.Add("cbf/max_speed", 1);
                }
                return true;
            }
        }
        
        if (statsRecorder != null && episodeLogger != null)
        {
            statsRecorder.Add("CBF/MaxSpeed", 0);
            episodeLogger.Add("cbf/max_speed", 0);
        }

        return false;
    }
}
