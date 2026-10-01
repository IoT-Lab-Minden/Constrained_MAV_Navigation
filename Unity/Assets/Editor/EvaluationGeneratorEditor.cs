using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(EvaluationGenerator))]
public class EvaluationGeneratorEditor : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        EvaluationGenerator evaluator = (EvaluationGenerator)target;

        if (GUILayout.Button("Start Evaluation"))
        {
            evaluator.StartEvaluation();
        }
    }
}