using UnityEngine;

public class PDControlAgent : ClassicAgenticController
{
    [Header("Control Gains")]
    public float Kp = 0.7f;   // Position correction
    public float Kd = 1.6f;   // Velocity damping

    [Header("Trajectory Time Adaptation")]
    public bool adaptiveTimeScaling = true;
    [Range(0.1f, 1.0f)] public float minTrajectoryTimeScale = 0.4f;
    [Range(0.1f, 1.0f)] public float maxTrajectoryTimeScale = 1.0f;
    [Min(0f)] public float errorForNoSlowdown = 0.25f;
    [Min(0.01f)] public float errorForFullSlowdown = 1.0f;

    private Vector3 lastTargetPos;
    private bool hasLastTargetPos = false;
    protected override ControllerState GenerateAction()
    {
        if (cachedGenerator == null)
            return new ControllerState();

        float controlDt = 1f / Mathf.Max(1, actionFrequencyHz);
        Vector3 currentPos = drone.GetDroneState().position;

        float trajectoryTimeScale = 1f;
        if (adaptiveTimeScaling)
        {
            Vector3 nominalTargetPos = cachedGenerator.EvaluatePosition(time, false);
            float positionError = Vector3.Distance(currentPos, nominalTargetPos);

            float slowDownAmount = Mathf.InverseLerp(errorForNoSlowdown, errorForFullSlowdown, positionError);
            trajectoryTimeScale = Mathf.Lerp(maxTrajectoryTimeScale, minTrajectoryTimeScale, slowDownAmount);
        }

        float effectiveDt = Mathf.Max(1e-4f, controlDt * trajectoryTimeScale);
        time += effectiveDt;

        Vector3 targetPos = cachedGenerator.EvaluatePosition(time);

        Vector3 targetVel = Vector3.zero;
        if (hasLastTargetPos)
        {
            targetVel = (targetPos - lastTargetPos) / effectiveDt;
        }
        else
        {
            hasLastTargetPos = true;
        }

        lastTargetPos = targetPos;

        Vector3 currentVel = drone.GetDroneState().linearVelocity;

        Vector3 ep = targetPos - currentPos;
        Vector3 ev = targetVel - currentVel;

        Vector3 v_des = Kp * ep + Kd * ev;
        Vector3 v_body = drone.transform.InverseTransformDirection(v_des);

        float lx = Mathf.Clamp(-v_body.z, -1f, 1f);
        float ly = Mathf.Clamp(v_body.x, -1f, 1f);
        float ry = Mathf.Clamp(v_body.y, -1f, 1f);

        double yaw = cachedGenerator.EvaluateYaw(time, currentPos);
        float targetYawDeg = (float)(yaw * Mathf.Rad2Deg) - 90f;

        float currentYaw = drone.GetDroneState().rotation.eulerAngles.y;
        float yawError = Mathf.DeltaAngle(currentYaw, targetYawDeg);

        float rx = Mathf.Clamp(yawError / 45f, -1f, 1f); // normalized

        return new ControllerState(lx, ly, rx, ry);
    }
}
