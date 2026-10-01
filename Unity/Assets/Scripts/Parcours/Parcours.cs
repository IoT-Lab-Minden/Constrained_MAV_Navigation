using System;
using System.Collections.Generic;
using UnityEngine;
#if IOTLAB_HELPERS
using IoTLab.Helpers;
#endif

public class Parcours : MonoBehaviour
{
    [Header("Parcours configuration")]
    public ParcoursGenerator.ParcoursType parcoursType;
    public Waypoint.Type waypointType;
    public float sizeModifier = 1.0f;
    public int waypointCount = 10;

    [Header("References")]
    public GameObject agent;
    private Vector3 startPosition;

    protected event Action OnWaypointReached;
    protected event Action OnParcoursFinished;

    protected event Action<int> OnWaypointHit;

    private List<Waypoint> wayPoints;
    private ParcoursGenerator generator;

    // index of current way point
    private int currentWayPoint;

#if IOTLAB_HELPERS
    private ApplicationSettings config;
#endif

    private static Material colorRed;
    private static Material colorGreen;
    private static Material colorDarkBlue;

    public void Awake()
    {
        ResetParcours(Vector3.zero);

        if (colorRed == null)
        {
            colorRed = new Material(Shader.Find("Unlit/Color"))
            {
                color = Color.red
            };
            colorGreen = new Material(Shader.Find("Unlit/Color"))
            {
                color = Color.green
            };
            colorDarkBlue = new Material(Shader.Find("Unlit/Color"))
            {
                color = new Color(0, 0, 0.5f)
            };
        }
    }

    public void OnRenderObject() {
        for (int i = 0; i < wayPoints.Count; i++)
        {
            Vector3 prevWaypointPosition = (i == 0) ? startPosition : wayPoints[i - 1].transform.position;

            Material lineMaterial = (currentWayPoint > i) ? colorRed : (currentWayPoint == i) ? colorGreen : colorDarkBlue;

            Camera cam = Camera.current ?? Camera.main;
            if (cam == null) return;

            GL.PushMatrix();
            GL.modelview = cam.worldToCameraMatrix;
            GL.LoadProjectionMatrix(cam.projectionMatrix);

            lineMaterial.SetPass(0);
            GL.Begin(GL.LINES);
            GL.Vertex(prevWaypointPosition);
            GL.Vertex(wayPoints[i].transform.position);
            GL.End();
            GL.PopMatrix();
        }

        if (waypointType == Waypoint.Type.COMPLEX) {
            if (currentWayPoint < wayPoints.Count)
            {
                Vector3 direction = Vector3.zero;
                ComplexWaypoint waypoint = (ComplexWaypoint) this.GetCurrentWayPoint();
                if (waypoint != null)
                {
                    switch (waypoint.droneOrientationType)
                    {
                        case DroneOrientationType.TOWARDS_WAYPOINT:
                            direction = Vector3.Normalize(waypoint.transform.position - agent.transform.position);
                            break;
                        case DroneOrientationType.TOWARDS_POINT:
                            direction = Vector3.Normalize(waypoint.droneOrientation - agent.transform.position);
                            break;
                        case DroneOrientationType.STATIC:
                            direction = waypoint.droneOrientation;
                            break;
                    }
                }

                direction.y = 0.0f;
                if (direction.magnitude > direction.normalized.magnitude * 0.1f) direction = direction.normalized * 0.1f;

                Material lineMaterial = colorRed;

                Camera cam = Camera.current ?? Camera.main;
                if (cam == null) return;

                GL.PushMatrix();
                GL.modelview = cam.worldToCameraMatrix;
                GL.LoadProjectionMatrix(cam.projectionMatrix);

                lineMaterial.SetPass(0);
                GL.Begin(GL.LINES);
                GL.Vertex(agent.transform.position);
                GL.Vertex(agent.transform.position + direction);
                GL.End();
                GL.PopMatrix();
            }
        }
    }

    private ParcoursGenerator GetParcoursGenerator() {
        switch (parcoursType) {
            case ParcoursGenerator.ParcoursType.STATIC:
                generator = new StaticParcoursGenerator(this, sizeModifier, waypointCount, waypointType, WaypointReachedEvent, WaypointHitEvent);
                break;
            case ParcoursGenerator.ParcoursType.CIRCLE:
                generator = new CircleParcoursGenerator(this, sizeModifier, waypointCount, waypointType, WaypointReachedEvent, WaypointHitEvent);
                break;
            case ParcoursGenerator.ParcoursType.RANDOM:
                generator = new RandomParcoursGenerator(this, sizeModifier, waypointCount, waypointType, WaypointReachedEvent, WaypointHitEvent);
                break;
        }

        return generator;
    }

    public void ResetParcours(Vector3 startPosition) {
        ParcoursGenerator generator = GetParcoursGenerator();
        wayPoints = generator.Generate(startPosition);
        for (int i = 0; i < wayPoints.Count; i++) {
            wayPoints[i].SetIndex(i);
        }

        this.startPosition = startPosition;
        currentWayPoint = 0;
        if (wayPoints.Count > 0)
            wayPoints[currentWayPoint].SetActiveWaypoint();
    }

    public Waypoint GetPreviousWayPoint() {
        if (currentWayPoint-1 >= wayPoints.Count || currentWayPoint-1 < 0) return null;

        return wayPoints[currentWayPoint-1];
    }

    public Waypoint GetSecondPreviousWaypoint()
    {
        if (currentWayPoint-2 >= wayPoints.Count || currentWayPoint-2 < 0) return null;

        return wayPoints[currentWayPoint-2];
    }

    public Waypoint GetCurrentWayPoint()
    {
        if (currentWayPoint >= wayPoints.Count) return null;

        return wayPoints[currentWayPoint];
    }

    public Waypoint GetNextWaypoint() {
        if (currentWayPoint+1 >= wayPoints.Count) return null;

        return wayPoints[currentWayPoint+1];
    }

    public List<Waypoint> GetWaypointList()
    {
        if (wayPoints == null) return new List<Waypoint>();
        return new List<Waypoint>(wayPoints);
    }

    public Vector3 GetLastWayPointPosition() {
        if (currentWayPoint == 0)
            return startPosition;
        if (currentWayPoint > wayPoints.Count)
            return wayPoints[wayPoints.Count].transform.position;

        return wayPoints[currentWayPoint-1].transform.position;
    }

    public Vector3 GetSecondLastWayPointPosition() {
        if (currentWayPoint <= 1)
            return startPosition;
        if (currentWayPoint >= wayPoints.Count)
            return wayPoints[wayPoints.Count-1].transform.position;

        return wayPoints[currentWayPoint-2].transform.position;
    }

    public Vector3 GetStartPosition() {
        return startPosition;
    }

    public int GetCurrentWayPointIndex() {
        return currentWayPoint;
    }

    public bool IsParcoursFinished()
    {
        return currentWayPoint >= wayPoints.Count;
    }

    public void SetOnWaypointReachedCallback(Action callback) {
        OnWaypointReached += callback;
    }

    public void SetOnParcoursFinishedCallback(Action callback) {
        OnParcoursFinished += callback;
    }

    public void SetOnWaypointHitCallback(Action<int> callback) {
        OnWaypointHit += callback;
    }

    public void WaypointHitEvent(object waypoint, Waypoint.WaypointReachedEvent e)
    {
        if (!(waypoint is Waypoint)) return;

        OnWaypointHit?.Invoke(((Waypoint) waypoint).GetIndex());
    }

    public void WaypointReachedEvent(object waypoint, Waypoint.WaypointReachedEvent e) {
        if (!(waypoint is Waypoint)) return;

        if ((Waypoint) waypoint != wayPoints[currentWayPoint]) {
            return;
        }

        if (agent != null && agent != e.other.gameObject) {
            // Wrong agent reached the waypoint, reset it
            wayPoints[currentWayPoint].SetActiveWaypoint();
            return;
        }

        OnWaypointReached?.Invoke();
        currentWayPoint++;
        if (currentWayPoint < wayPoints.Count) {
            wayPoints[currentWayPoint].SetActiveWaypoint();
        } else {
            OnParcoursFinished?.Invoke();
        }
    }
}
