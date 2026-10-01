using UnityEngine;

public class CorridorCBF : ControlBarrierFunction
{
    private float alpha1 = 0.5f;

    public CorridorCBF()
    {
        SetSafetyMargin(0.5f);
    }

    public override bool ApplyCBF(ref ControllerState controllerState,
                          DroneState drone,
                          ParcoursState parcours)
    {
        if (parcours.currentWaypoint == null || parcours.currentWaypoint is not ComplexWaypoint)
        {
            if (statsRecorder != null && episodeLogger != null)
            {
                statsRecorder.Add("CBF/Corridor", 0);
                episodeLogger.Add("cbf/corridor", 0);
            }
            return false; // CorridorCBF only defined for ComplexWaypoints
        }
        
        ComplexWaypoint currentWaypoint =
            (ComplexWaypoint)parcours.currentWaypoint;

        Vector3 p0 = parcours.previousWaypoint != null ? parcours.previousWaypoint.transform.position : parcours.startPosition;
        Vector3 p  = drone.position;
        Vector3 v  = drone.linearVelocity;


        Vector3 t =
            (currentWaypoint.transform.position - p0).normalized; // Corridor axis (from previous waypoint to current waypoint)
        // Velocity component along the corridor axis
        Vector3 vParallel = Vector3.Dot(v, t) * t;
        // Lateral velocity component, i.e. movement away from / toward the corridor centerline
        Vector3 vPerp = v - vParallel;


        // Projection on Corridor axis
        Vector3 pProj = p0 + Vector3.Dot(p - p0, t) * t; // Projection of p on the line through p0 in the direction of t
        Vector3 e = p - pProj; // Distance vector from p to the Corridor axis

        float d = e.magnitude; // Distance from p to the Corridor axis

        if (d < 1e-6f)
        {
            if (statsRecorder != null && episodeLogger != null)
            {
                statsRecorder.Add("CBF/Corridor", 0);
                episodeLogger.Add("cbf/corridor", 0);
            }
            return false; // If the drone is almost on the Corridor axis, the normal is not defined -> No intervention needed
        }

        Vector3 n = e / d; // Normal vector of the Corridor

        float r =
            currentWaypoint.flightRadius
            * (1-safetyMargin); // Final radius of the Corridor (including safety margin)

        // --- HOCBF ---
        float h0 = r * r - d * d;                 // Barrier function, positional part
        float dh0 = -2f * Vector3.Dot(e, vPerp);      // 1. Derivative (Determines, if the drone is moving away from the Corridor axis or not)

        float h1 = dh0 + alpha1 * h0;

        // Only intervene if the HOCBF is violated
        if (h1 >= 0f)
        {
            if (statsRecorder != null && episodeLogger != null)
            {
                statsRecorder.Add("CBF/Corridor", 0);
                episodeLogger.Add("cbf/corridor", 0);
            }
            return false;
        }

        Vector3 uLocal = new Vector3(controllerState.ly,
                    controllerState.ry,
                   -controllerState.lx);
        Quaternion yawRotation = Quaternion.Euler(0f, drone.rotation.eulerAngles.y, 0f);
        Vector3 uWorld = yawRotation * uLocal;

        float uRad = Vector3.Dot(uWorld, n);
        bool modified = false;

        if (uRad > 0f)
        {
            uWorld -= uRad * n;
            modified = true;
        }

        if (!modified)
        {
            if (statsRecorder != null && episodeLogger != null)
            {
                statsRecorder.Add("CBF/Corridor", 0);
                episodeLogger.Add("cbf/corridor", 0);
            }
            return false;
        }

        uLocal = Quaternion.Inverse(yawRotation) * uWorld;

        controllerState.ly = Mathf.Clamp(uLocal.x, -1f, 1f);
        controllerState.ry = Mathf.Clamp(uLocal.y, -1f, 1f);
        controllerState.lx = Mathf.Clamp(-uLocal.z, -1f, 1f);
        
        if (statsRecorder != null && episodeLogger != null)
        {
            statsRecorder.Add("CBF/Corridor", 1);
            episodeLogger.Add("cbf/corridor", 1);
        }

        return true;
    }
}
