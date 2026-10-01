using System;
using System.Collections.Generic;
using UnityEngine;

public class StaticParcoursGenerator : ParcoursGenerator
{
    public StaticParcoursGenerator(Parcours parent, float size, int waypointCount, Waypoint.Type waypointType, Action<object, Waypoint.WaypointReachedEvent> waypointCallback, Action<object, Waypoint.WaypointReachedEvent> waypointHitCallback)
    : base(parent, size, waypointCount, waypointType, waypointCallback, waypointHitCallback) {    
        type = ParcoursType.STATIC;
    }

    public override List<Waypoint> Generate(Vector3 startPosition)
    {
        var waypoints = new List<Waypoint>();
        for (int i = 0; i < parent.transform.childCount; i++) {
            Waypoint waypoint = parent.transform.GetChild(i).GetComponent<Waypoint>();
            waypoint.ResetWaypoint();
            waypoint.SetWaypointHitCallback(waypointHitCallback);
            waypoint.SetWaypointReachedCallback(waypointCallback);

            if (i == 0)
            {
                if (parent.transform.childCount == 1)
                {
                    waypoint.transform.rotation = CalculateOrientation(
                        startPosition,
                        parent.transform.GetChild(i).transform.position,
                        parent.transform.GetChild(i).transform.position + (parent.transform.GetChild(i).transform.position - startPosition)
                    );
                } else
                {
                    waypoint.transform.rotation = CalculateOrientation(
                        startPosition,
                        parent.transform.GetChild(i).transform.position,
                        parent.transform.GetChild(i+1).transform.position
                    );
                }
            }
            else if (i == parent.transform.childCount - 1)
            {
                waypoint.transform.rotation = CalculateOrientation(
                    parent.transform.GetChild(i - 1).transform.position,
                    parent.transform.GetChild(i).transform.position,
                    parent.transform.GetChild(i).transform.position + (parent.transform.GetChild(i).transform.position - parent.transform.GetChild(i - 1).transform.position)
                );
            }
            else
            {
                waypoint.transform.rotation = CalculateOrientation(
                    parent.transform.GetChild(i-1).transform.position,
                    parent.transform.GetChild(i).transform.position,
                    parent.transform.GetChild(i+1).transform.position
                );
            }

            waypoints.Add(waypoint);
        }

        return waypoints;
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
