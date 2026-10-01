using UnityEngine;
using System.Collections.Generic;

public class MPCControlAgent : ClassicAgenticController
{
    [Header("MPC Prediction")]
    [Range(4, 30)] public int horizonSteps = 10;
    [Min(0.01f)] public float predictionDt = 0.1f;

    [Header("MPC Search")]
    [Min(0.01f)] public float fineSearchRadius = 0.35f;
    [Min(0.01f)] public float fineSearchStep = 0.175f;

    [Header("Cost Weights")]
    [Min(0f)] public float positionWeight = 6.0f;
    [Min(0f)] public float velocityWeight = 1.0f;
    [Min(0f)] public float controlEffortWeight = 0.15f;
    [Min(0f)] public float smoothnessWeight = 0.4f;

    [Header("Trajectory Time Adaptation")]
    public bool adaptiveTimeScaling = true;
    [Range(0.1f, 1.0f)] public float minTrajectoryTimeScale = 0.4f;
    [Range(0.1f, 1.0f)] public float maxTrajectoryTimeScale = 1.0f;
    [Min(0f)] public float errorForNoSlowdown = 0.25f;
    [Min(0.01f)] public float errorForFullSlowdown = 1.0f;

    [Header("Yaw")]
    [Range(10f, 90f)] public float yawNormalizationDeg = 45f;
    private Vector3 lastTargetPos;
    private bool hasLastTargetPos = false;
    private Vector3 lastBodyCommand = Vector3.zero;
    private Transform predictionTransform;

    protected override ControllerState GenerateAction()
    {
        if (cachedGenerator == null)
            return new ControllerState();

        float controlDt = 1f / Mathf.Max(1, actionFrequencyHz);
        Vector3 currentPos = drone.GetDroneState().position;
        Vector3 currentVel = drone.GetDroneState().linearVelocity;

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

        Vector3 targetPosNow = cachedGenerator.EvaluatePosition(time);
        Vector3 targetVelNow = Vector3.zero;
        if (hasLastTargetPos)
        {
            targetVelNow = (targetPosNow - lastTargetPos) / effectiveDt;
        }
        else
        {
            hasLastTargetPos = true;
        }
        lastTargetPos = targetPosNow;

        Vector3 bestBodyCommand = FindBestBodyCommand(drone, currentPos, currentVel);
        lastBodyCommand = bestBodyCommand;

        float lx = Mathf.Clamp(-bestBodyCommand.z, -1f, 1f);
        float ly = Mathf.Clamp(bestBodyCommand.x, -1f, 1f);
        float ry = Mathf.Clamp(bestBodyCommand.y, -1f, 1f);

        double yaw = cachedGenerator.EvaluateYaw(time, currentPos);
        float targetYawDeg = (float)(yaw * Mathf.Rad2Deg) - 90f;
        float currentYaw = drone.GetDroneState().rotation.eulerAngles.y;
        float yawError = Mathf.DeltaAngle(currentYaw, targetYawDeg);
        float rx = Mathf.Clamp(yawError / yawNormalizationDeg, -1f, 1f);

        /*Debug.Log(
            "MPC t=" + time.ToString("F2") +
            " ePos=" + (targetPosNow - currentPos).magnitude.ToString("F2") +
            " eVel=" + (targetVelNow - currentVel).magnitude.ToString("F2") +
            " cmd=" + new Vector4(lx, ly, rx, ry).ToString("F2")
        );*/

        return new ControllerState(lx, ly, rx, ry);
    }

    private Vector3 FindBestBodyCommand(Drone drone, Vector3 currentPos, Vector3 currentVel)
    {
        float dt = Mathf.Max(0.01f, predictionDt);
        Vector3[] coarseCandidates = BuildCoarseCandidateCommands();
        Vector3 coarseBest = EvaluateBestCandidate(drone, currentPos, currentVel, dt, coarseCandidates);

        Vector3[] fineCandidates = BuildFineCandidateCommands(coarseBest);
        return EvaluateBestCandidate(drone, currentPos, currentVel, dt, fineCandidates);
    }

    private Vector3 EvaluateBestCandidate(
        Drone drone,
        Vector3 currentPos,
        Vector3 currentVel,
        float dt,
        Vector3[] candidates)
    {
        float bestCost = float.PositiveInfinity;
        Vector3 bestCommand = Vector3.zero;

        float actionDelay = drone.GetActionDelayEstimate();

        for (int i = 0; i < candidates.Length; i++)
        {
            Vector3 cmd = candidates[i];
            float cost = RolloutCost(drone, currentPos, currentVel, cmd, dt, actionDelay);
            if (cost < bestCost)
            {
                bestCost = cost;
                bestCommand = cmd;
            }
        }

        return bestCommand;
    }

    private float RolloutCost(Drone drone, Vector3 startPos, Vector3 startVel, Vector3 bodyCommand, float dt, float actionDelay)
    {
        EnsurePredictionTransform();

        SimulationModel modelClone = drone.CreateSimulationModelClone();
        predictionTransform.position = startPos;
        predictionTransform.rotation = drone.GetDroneState().rotation;

        Rigidbody predictionRb = predictionTransform.GetComponent<Rigidbody>();
        if (predictionRb != null)
        {
            if (predictionRb.isKinematic)
            {
                predictionRb.isKinematic = false;
            }

            predictionRb.linearVelocity = startVel;
            predictionRb.angularVelocity = Vector3.zero;
        }

        modelClone.InitializePredictionState(predictionTransform, startVel, drone.GetDroneState().angularVelocity);

        float stepDt = Mathf.Max(Time.fixedDeltaTime, 1e-4f);
        int simulationSubsteps = Mathf.Max(1, Mathf.RoundToInt(dt / stepDt));
        float horizonStepDt = simulationSubsteps * stepDt;

        float elapsed = 0f;
        float cost = 0f;
        Vector3 prevTargetPos = cachedGenerator.EvaluatePosition(time, false);

        float rxZero = 0f;
        ControllerState cmdState = new ControllerState(-bodyCommand.z, bodyCommand.x, rxZero, bodyCommand.y);
        ControllerState prevCmdState = new ControllerState(-lastBodyCommand.z, lastBodyCommand.x, rxZero, lastBodyCommand.y);

        for (int k = 0; k < horizonSteps; k++)
        {
            for (int sub = 0; sub < simulationSubsteps; sub++)
            {
                ControllerState activeCmd = (elapsed >= actionDelay) ? cmdState : prevCmdState;
                modelClone.UpdateOrientation(activeCmd, predictionTransform);
                modelClone.UpdatePosition(activeCmd, predictionTransform);
                elapsed += stepDt;
            }

            float tk = time + elapsed;
            Vector3 targetPos = cachedGenerator.EvaluatePosition(tk, false);
            Vector3 targetVel = (targetPos - prevTargetPos) / horizonStepDt;
            prevTargetPos = targetPos;

            Vector3 predPos = predictionTransform.position;
            Vector3 predVel = modelClone.GetLinearVelocity();

            Vector3 ePos = predPos - targetPos;
            Vector3 eVel = predVel - targetVel;

            cost += positionWeight * ePos.sqrMagnitude;
            cost += velocityWeight * eVel.sqrMagnitude;
        }

        cost += controlEffortWeight * bodyCommand.sqrMagnitude;
        cost += smoothnessWeight * (bodyCommand - lastBodyCommand).sqrMagnitude;

        return cost;
    }

    private static Vector3[] BuildCoarseCandidateCommands()
    {
        List<Vector3> candidates = new List<Vector3>
        {
            Vector3.zero
        };

        float[] magnitudes = { 1f, 0.7f, 0.3f };

        for (int m = 0; m < magnitudes.Length; m++)
        {
            float a = magnitudes[m];
            float[] levels = { -a, 0f, a };

            for (int ix = 0; ix < levels.Length; ix++)
            {
                for (int iy = 0; iy < levels.Length; iy++)
                {
                    for (int iz = 0; iz < levels.Length; iz++)
                    {
                        Vector3 cmd = new Vector3(levels[ix], levels[iy], levels[iz]);
                        if (cmd.sqrMagnitude > 0f)
                        {
                            candidates.Add(cmd);
                        }
                    }
                }
            }
        }

        return candidates.ToArray();
    }

    private Vector3[] BuildFineCandidateCommands(Vector3 center)
    {
        float radius = Mathf.Max(0.01f, fineSearchRadius);
        float step = Mathf.Max(0.01f, fineSearchStep);

        int stepsPerAxis = Mathf.Max(1, Mathf.RoundToInt(radius / step));
        int side = 2 * stepsPerAxis + 1;
        Vector3[] candidates = new Vector3[side * side * side + 1];

        int idx = 0;
        candidates[idx++] = new Vector3(
            Mathf.Clamp(center.x, -1f, 1f),
            Mathf.Clamp(center.y, -1f, 1f),
            Mathf.Clamp(center.z, -1f, 1f)
        );

        for (int ix = -stepsPerAxis; ix <= stepsPerAxis; ix++)
        {
            for (int iy = -stepsPerAxis; iy <= stepsPerAxis; iy++)
            {
                for (int iz = -stepsPerAxis; iz <= stepsPerAxis; iz++)
                {
                    Vector3 offset = new Vector3(ix * step, iy * step, iz * step);
                    Vector3 candidate = center + offset;
                    Vector3 cmd = new Vector3(
                        Mathf.Clamp(candidate.x, -1f, 1f),
                        Mathf.Clamp(candidate.y, -1f, 1f),
                        Mathf.Clamp(candidate.z, -1f, 1f)
                    );
                    candidates[idx++] = cmd;
                }
            }
        }

        return candidates;
    }

    private void EnsurePredictionTransform()
    {
        if (predictionTransform != null)
            return;

        GameObject predictionObject = new GameObject("MPC_PredictionDrone");
        predictionObject.hideFlags = HideFlags.HideAndDontSave;
        Rigidbody rb = predictionObject.AddComponent<Rigidbody>();
        rb.useGravity = false;
        rb.isKinematic = false;
        predictionTransform = predictionObject.transform;
    }

    public override void OnDestroy()
    {
        base.OnDestroy();
        if (predictionTransform != null)
        {
            Destroy(predictionTransform.gameObject);
        }
    }
}
