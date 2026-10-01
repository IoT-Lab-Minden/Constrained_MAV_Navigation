using System;
using System.Collections.Generic;
using UnityEngine;
using Random = UnityEngine.Random;

public class RandomParcoursGenerator : ParcoursGenerator
{
    private List<Vector3> positions;

    public RandomParcoursGenerator(Parcours parent, float size, int waypointCount, Waypoint.Type waypointType, Action<object, Waypoint.WaypointReachedEvent> waypointCallback, Action<object, Waypoint.WaypointReachedEvent> waypointHitCallback)
    : base(parent, size, waypointCount, waypointType, waypointCallback, waypointHitCallback) {    
        type = ParcoursType.RANDOM;
    }

    public override List<Waypoint> Generate(Vector3 startPosition)
    {
        ClearWaypointObjects();

        positions = new List<Vector3> {
            CalculatePosition(),
            CalculatePosition()
        };

        var waypoints = new List<Waypoint>();
        for (int i = 0; i < waypointCount; i++) {
            positions.Add(CalculatePosition());
            var orientation = CalculateOrientation(positions[positions.Count-3], positions[positions.Count-2], positions[positions.Count-1]);

            waypoints.Add(InstantiateWaypoint(positions[positions.Count-2], orientation));
            if (waypointType == Waypoint.Type.COMPLEX) {
                GenerateRandomComplexWaypointData(waypoints[i]);
            }
        }

        return waypoints;
    }

    private void GenerateRandomComplexWaypointData(Waypoint waypoint)
    {
        float radiusValue = 1.0f;
        for (int i = 0; i < 6; i++) {
            radiusValue *= (float) UnityEngine.Random.value;
        }
        float flightRadius = (float) radiusValue + 0.2f;
        
        float maxSpeed = 1.109354f - (float) UnityEngine.Random.value * (1.109354f - 0.2f);

        DroneOrientationType type = DroneOrientationType.STATIC;
        Vector3 droneOrientation = Vector3.zero;
        switch (UnityEngine.Random.Range(0, 3)) {
            case 0:
                type = DroneOrientationType.STATIC;
                droneOrientation = new Vector3(
                    (float)UnityEngine.Random.value-0.5f,
                    0.0f,
                    (float)UnityEngine.Random.value-0.5f
                ).normalized;
                break;
            case 1:
                type = DroneOrientationType.TOWARDS_POINT;
                droneOrientation = waypoint.transform.position + new Vector3(
                    (float) UnityEngine.Random.value * 0.5f * (UnityEngine.Random.Range(0, 2) == 0 ? -1 : 1),
                    0.0f,
                    (float) UnityEngine.Random.value * 0.5f * (UnityEngine.Random.Range(0, 2) == 0 ? -1 : 1)
                );
                droneOrientation.y = 0.0f;
                break;
            case 2:
                type = DroneOrientationType.TOWARDS_WAYPOINT;
                droneOrientation = Vector3.zero;
                break;
        }

        (waypoint as ComplexWaypoint).SetDroneRequirements(flightRadius, maxSpeed, type, droneOrientation);
    }

    private Vector3 CalculatePosition() {
        return new Vector3((float)(UnityEngine.Random.value * 2 - 1) * size, (float)UnityEngine.Random.value * size + 0.1f,
                (float)(UnityEngine.Random.value * 2 - 1) * size);
    }

    private Quaternion CalculateOrientation(Vector3 prev, Vector3 current, Vector3 next) {
        Vector3 dirToPrev = (current - prev).normalized;
        Vector3 dirToNext = (next - current).normalized;

        if (dirToPrev == Vector3.zero) return Quaternion.LookRotation(dirToNext);
        if (dirToNext == Vector3.zero) return Quaternion.LookRotation(dirToPrev);

        Vector3 averageDir = ((dirToPrev + dirToNext) * 0.5f).normalized;

        return Quaternion.LookRotation(averageDir) * Quaternion.Euler(0, -90, 0);
    }
}
