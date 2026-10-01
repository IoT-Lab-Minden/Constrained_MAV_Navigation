using System.Collections.Generic;
using UnityEngine;

public class EvaluationGenerator : MonoBehaviour
{
    public int currentParcours = 0;

    public Vector3 parcoursBasePosition = Vector3.zero;

    [Header("Prefabs")]
    [Tooltip("Disable the agent mono behavior in the agent prefab for evaluation, as it will be activated by the evaluation script at the right time.")]
    public GameObject agentPrefab;
    public List<GameObject> parcoursPrefabs;
    public GameObject emptyEnvironmentPrefab;

    private Parcours parcours;
    private RLAgenticController RLagent;
    private ClassicAgenticController classicAgent;

    private Vector3 lastBasePosition = Vector3.up;

    void Start()
    {
        GenerateEnvironment();
    }

    void Update()
    {
        if (parcours)
        {
            if (parcoursBasePosition != lastBasePosition)
            {
                parcours.transform.position = parcoursBasePosition;
                lastBasePosition = parcoursBasePosition;
            }
        }

        if (Input.GetKeyDown(KeyCode.B))
        {
            if (RLagent != null) {
                RLagent.enabled = false;
                RLagent.GetDrone().SetControllerState(new ControllerState(0, 0, 0, 0));
            } else if (classicAgent != null) {
                classicAgent.enabled = false;
                classicAgent.GetDrone().SetControllerState(new ControllerState(0, 0, 0, 0));
            }
            Debug.Log("Disabled agent");
        }
    }

    void GenerateEnvironment()
    {
        GameObject envObj = Instantiate(emptyEnvironmentPrefab, transform);
        parcours = Instantiate(parcoursPrefabs[currentParcours], envObj.transform).GetComponent<Parcours>();

        GameObject agent = Instantiate(agentPrefab, envObj.transform);
        if (agent.GetComponent<RLAgenticController>() != null) {
            RLagent = agent.GetComponent<RLAgenticController>();
            RLagent.parcours = parcours;
            parcours.agent = RLagent.gameObject;
            RLagent.enabled = false;
        } else if (agent.GetComponent<ClassicAgenticController>() != null) {
            classicAgent = agent.GetComponent<ClassicAgenticController>();
            classicAgent.parcours = parcours;
            parcours.agent = classicAgent.gameObject;
            classicAgent.enabled = false;
            classicAgent.InitParcours();

            parcours.gameObject.AddComponent<TrajectoryVisualizer>();
            parcours.GetComponent<TrajectoryVisualizer>().parcours = parcours;
            parcours.GetComponent<TrajectoryVisualizer>().drone = classicAgent.GetDrone();
            parcours.GetComponent<TrajectoryVisualizer>().agent = classicAgent;
        }

        if (RLagent != null) {
            RLagent.inference = true;
            RLagent.episodeLogger.loggingEnabled = true;
        } else if (classicAgent != null) {
            classicAgent.episodeLogger.loggingEnabled = true;
        }

        if (RLagent != null) {
            GameObject.Find("KeyboardController").GetComponent<KeyboardController>().drone = RLagent.GetDrone();
        } else if (classicAgent != null) {
            GameObject.Find("KeyboardController").GetComponent<KeyboardController>().drone = classicAgent.GetDrone();
        }
    }

    public void StartEvaluation()
    {
        if (RLagent != null) {
            parcours.ResetParcours(RLagent.GetDrone().transform.position);
            RLagent.enabled = true;
        } else if (classicAgent != null) {
            parcours.ResetParcours(classicAgent.GetDrone().transform.position);
            classicAgent.enabled = true;
        }
    }
}