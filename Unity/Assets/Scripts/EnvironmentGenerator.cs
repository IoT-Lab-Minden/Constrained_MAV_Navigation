using System.Collections.Generic;
using Unity.MLAgents;
using UnityEngine;

public class EnvironmentGenerator : MonoBehaviour
{
    [Header("Settings")]
    public int environmentCount;

    [Header("Prefabs")]
    public GameObject agentPrefab;
    public List<GameObject> parcoursPrefabs;
    public GameObject emptyEnvironmentPrefab;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        GenerateEnvironments();
    }

    void GenerateEnvironments() {
        int envCount = 0;
        for (int env = 0; env < environmentCount / parcoursPrefabs.Count; env++) {
            for (int parcours = 0; parcours < parcoursPrefabs.Count; parcours++) {
                if (envCount < environmentCount) {
                    GameObject envObj = Instantiate(emptyEnvironmentPrefab, this.transform);
                    envObj.transform.position = new Vector3(env*5, 0, parcours*5);

                    RLAgenticController agent = Instantiate(agentPrefab, envObj.transform).GetComponent<RLAgenticController>();
                    agent.parcours = Instantiate(parcoursPrefabs[parcours], envObj.transform).GetComponent<Parcours>();
                }
                envCount++;
            }
        }
    }
}
