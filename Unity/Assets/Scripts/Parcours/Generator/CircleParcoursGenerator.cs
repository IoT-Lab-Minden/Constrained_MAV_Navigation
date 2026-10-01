using System;
using System.Collections.Generic;
using UnityEngine;
using Random = UnityEngine.Random;

public class CircleParcoursGenerator : ParcoursGenerator
{
    bool complexInitialized = false;

    float flightRadius;
    float maxSpeed;
    DroneOrientationType orientationType = DroneOrientationType.STATIC;
    Vector3 droneOrientation = Vector3.zero;

    public CircleParcoursGenerator(Parcours parent, float size, int waypointCount, Waypoint.Type waypointType, Action<object, Waypoint.WaypointReachedEvent> waypointCallback, Action<object, Waypoint.WaypointReachedEvent> waypointHitCallback)
    : base(parent, size, waypointCount, waypointType, waypointCallback, waypointHitCallback) {    
        type = ParcoursType.CIRCLE;
    }

    public override List<Waypoint> Generate(Vector3 startPosition)
    {
        ClearWaypointObjects();

        float baseAngle = (float) UnityEngine.Random.value * 360.0f;
        int direction = UnityEngine.Random.Range(0, 2) == 0 ? 1 : -1;
        float height = (float) UnityEngine.Random.value * size + 0.1f;
        float seed = (float) UnityEngine.Random.value;

        var waypoints = new List<Waypoint>();
        for (int i = 0; i < waypointCount; i++) {
            var position = CalculatePosition(i, baseAngle, direction, height);
            
            var prevPosition = (i == 0) ? startPosition : CalculatePosition(i-1, baseAngle, direction, height);
            var nextPosition = CalculatePosition(i+1, baseAngle, direction, height);
            var orientation = CalculateOrientation(prevPosition, position, nextPosition);

            waypoints.Add(InstantiateWaypoint(position, orientation));
            if (waypointType == Waypoint.Type.COMPLEX) {
                SetRandomizedComplexWaypointData(waypoints[i], parent.transform.position, seed);
            }
        }

        complexInitialized = false;
        return waypoints;
    }

    private void SetRandomizedComplexWaypointData(Waypoint waypoint, Vector3 basePosition, float seed)
    {
        if (!complexInitialized) {
            float radiusValue = 1.0f;
            for (int i = 0; i < 6; i++) {
                radiusValue *= (float) UnityEngine.Random.value;
            }
            flightRadius = (float) radiusValue + 0.2f;

            maxSpeed = 1.109354f - (float) UnityEngine.Random.value * (1.109354f - 0.1f);

            orientationType = DroneOrientationType.STATIC;
            droneOrientation = Vector3.zero;
            switch (UnityEngine.Random.Range(0, 3)) {
                case 0:
                    orientationType = DroneOrientationType.STATIC;
                    droneOrientation = new Vector3(
                        (float)UnityEngine.Random.value-0.5f,
                        0.0f,
                        (float)UnityEngine.Random.value-0.5f
                    ).normalized;
                    break;
                case 1:
                    orientationType = DroneOrientationType.TOWARDS_POINT;
                    droneOrientation = basePosition + new Vector3(
                        (float) UnityEngine.Random.value * 0.5f * (UnityEngine.Random.Range(0, 2) == 0 ? -1 : 1),
                        0.0f,
                        (float) UnityEngine.Random.value * 0.5f * (UnityEngine.Random.Range(0, 2) == 0 ? -1 : 1)
                    );
                    droneOrientation.y = 0.0f;
                    break;
                case 2:
                    orientationType = DroneOrientationType.TOWARDS_WAYPOINT;
                    droneOrientation = Vector3.zero;
                    break;
            }
            complexInitialized = true;
        }

        (waypoint as ComplexWaypoint).SetDroneRequirements(flightRadius, maxSpeed, orientationType, droneOrientation);
    }

    private Vector3 CalculatePosition(int positionIndex, float baseAngle, int direction, float height) {
        var angle = ((360.0f / waypointCount * positionIndex) + baseAngle) % 360 * direction;
        var arcAngle = (float) (Math.PI * angle / 180.0f);

        var x = (float) Math.Sin(arcAngle);
        var z = (float) Math.Cos(arcAngle);

        return new Vector3(x*size, height, z*size);
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
