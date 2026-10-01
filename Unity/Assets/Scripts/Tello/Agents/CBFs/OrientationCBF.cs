using UnityEngine;

public class OrientationCBF : ControlBarrierFunction
{
    private float controllerGain = 1.5f;
    
    // Helper: compute the horizontal target direction and distance r to the target
    // Returns a horizontal (y = 0) normalized direction when r > eps, otherwise Vector3.zero.
    private Vector3 GetHorizontalTargetDirection(DroneState drone, ComplexWaypoint waypoint, out float r)
    {
        Vector3 targetDirection = Vector3.zero;

        switch (waypoint.droneOrientationType)
        {
            case DroneOrientationType.TOWARDS_WAYPOINT:
                targetDirection = waypoint.transform.position - drone.position;
                break;
            case DroneOrientationType.TOWARDS_POINT:
                targetDirection = waypoint.droneOrientation - drone.position;
                break;
            case DroneOrientationType.STATIC:
                targetDirection = waypoint.droneOrientation;
                break;
        }

        targetDirection.y = 0f;
        r = targetDirection.magnitude;
        if (r > 1e-6f)
        {
            targetDirection.Normalize();
        }
        else
        {
            targetDirection = Vector3.zero;
        }

        return targetDirection;
    }
    public OrientationCBF()
    {
        SetSafetyMargin(10);
    }
    
    public override bool ApplyCBF(ref ControllerState controllerState, DroneState drone, ParcoursState parcours)
    {
        if (parcours.currentWaypoint == null || parcours.currentWaypoint is not ComplexWaypoint)
        {
            if (statsRecorder != null && episodeLogger != null)
            {
                statsRecorder.Add("CBF/Orientation", 0);
                episodeLogger.Add("cbf/orientation", 0);
            }
            return false; // OrientationCBF only defined for ComplexWaypoints
        }

        // theta approximated from current angular velocity (rotation) and
        // change of target direction due to linear velocity (translation).
        float theta = GetOrientationDeviation(drone, parcours.currentWaypoint);

        float uNominal = controllerState.rx;

        // barrier
        float h = safetyMargin - Mathf.Abs(theta);

        // direction
        float s = Mathf.Sign(theta);

        // Parameters
        float gamma = controllerGain;
        float ku = 98.0f;

        // CBF limit
        float uLimitRaw = (gamma / ku) * h;
        float uLimit = uLimitRaw / (1f + Mathf.Abs(uLimitRaw));

        float uSafe;

        if (s < 0f)
        {
            // u <= uLimit
            uSafe = Mathf.Min(uNominal, uLimit);
        }
        else if (s > 0f)
        {
            // u >= -uLimit
            uSafe = Mathf.Max(uNominal, -uLimit);
        }
        else
        {
            uSafe = uNominal;
        }
        
        controllerState.rx = Mathf.Clamp(uSafe, -1f, 1f);

        if (h < 0f)
        {
            if (statsRecorder != null && episodeLogger != null)
            {
                statsRecorder.Add("CBF/Orientation", 1);
                episodeLogger.Add("cbf/orientation", 1);
            }
            return true;
        }
        else if (statsRecorder != null && episodeLogger != null)
        {
            statsRecorder.Add("CBF/Orientation", 0);
            episodeLogger.Add("cbf/orientation", 0);
        }

        return false;
    }

    private float GetOrientationDeviation(DroneState drone, ComplexWaypoint waypoint)
    {
        Vector3 droneOrientation = Quaternion.AngleAxis(90f, Vector3.up) * drone.forward;
        droneOrientation.y = 0f;
        droneOrientation.Normalize();

        float r;
        Vector3 targetDirection = GetHorizontalTargetDirection(drone, waypoint, out r);
        if (r <= 1e-6f)
        {
            // If target is degenerate, use drone orientation so signed angle becomes zero
            targetDirection = Quaternion.AngleAxis(90f, Vector3.up) * drone.forward;
            targetDirection.y = 0f;
            targetDirection.Normalize();
        }

        float signedAngle = Vector3.SignedAngle(
            droneOrientation,
            targetDirection,
            Vector3.up
        );

        return signedAngle;
    }
}
