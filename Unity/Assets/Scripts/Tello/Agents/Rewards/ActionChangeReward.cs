using Unity.MLAgents.Actuators;
using UnityEngine;

public class ActionChangeReward : Reward
{
    private ActionBuffers previousActions;

    public ActionChangeReward(float weight) : base(weight, "ActionChange")
    {
        previousActions = new ActionBuffers(new ActionSegment<float>(new float[4]), new ActionSegment<int>(new int[0]));
    }

    public override float CalculateReward(RLAgenticController agent, ActionBuffers actions, DroneState droneState, ParcoursState parcoursState)
    {
        /*float div0 = Mathf.Abs(agent.ActionToInputContinuous(actions.ContinuousActions[0]) - agent.ActionToInputContinuous(previousActions.ContinuousActions[0]));
        float div1 = Mathf.Abs(agent.ActionToInputContinuous(actions.ContinuousActions[1]) - agent.ActionToInputContinuous(previousActions.ContinuousActions[1]));
        float div2 = Mathf.Abs(agent.ActionToInputContinuous(actions.ContinuousActions[2]) - agent.ActionToInputContinuous(previousActions.ContinuousActions[2]));
        float div3 = Mathf.Abs(agent.ActionToInputContinuous(actions.ContinuousActions[3]) - agent.ActionToInputContinuous(previousActions.ContinuousActions[3]));

        return -1 * (div0 + div1 + div2 + div3) / 4;*/

        Vector4 previousActionVector = new Vector4(
            agent.ActionToInputContinuous(previousActions.ContinuousActions[0]),
            agent.ActionToInputContinuous(previousActions.ContinuousActions[1]),
            agent.ActionToInputContinuous(previousActions.ContinuousActions[2]),
            agent.ActionToInputContinuous(previousActions.ContinuousActions[3]));
        Vector4 currentActionVector = new Vector4(
            agent.ActionToInputContinuous(actions.ContinuousActions[0]),
            agent.ActionToInputContinuous(actions.ContinuousActions[1]),
            agent.ActionToInputContinuous(actions.ContinuousActions[2]),
            agent.ActionToInputContinuous(actions.ContinuousActions[3]));
            
        float actionChangeMagnitude = Vector4.Distance(previousActionVector, currentActionVector);
        previousActions = actions;
        return -1 * actionChangeMagnitude;
    }
}
