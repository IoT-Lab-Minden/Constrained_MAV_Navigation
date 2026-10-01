using System;
using UnityEngine;

class WaypointBuilder {

    public static Waypoint GenerateWaypoint(float flightRadius, float maxSpeed, DroneOrientationType droneOrientationType, Vector3 droneOrientation) {
        return GenerateWaypoint(Waypoint.Type.COMPLEX, flightRadius, maxSpeed, droneOrientationType, droneOrientation);
    }

    public static Waypoint GenerateWaypoint(Waypoint.Type type) {
        switch (type) {
            case Waypoint.Type.COMPLEX:
                return GenerateComplexWaypoint();
            default:
                return null;
        }
    }

    public static Waypoint GenerateWaypoint(Waypoint.Type type, float flightRadius, float maxSpeed, DroneOrientationType droneOrientationType, Vector3 droneOrientation) {
        switch (type) {
            case Waypoint.Type.COMPLEX:
                return GenerateComplexWaypoint(flightRadius, maxSpeed, droneOrientationType, droneOrientation);
            default:
                return null;
        }
    }
    
    private static Waypoint GenerateComplexWaypoint() {
        GameObject obj = new GameObject();
        obj.name = "ComplexWaypoint";
        obj.tag = "Waypoint";
        obj.AddComponent(typeof(SphereCollider));
        obj.GetComponent<SphereCollider>().isTrigger = true;
        obj.AddComponent(typeof(BoxCollider));
        obj.GetComponent<BoxCollider>().isTrigger = true;
        ComplexWaypoint waypoint = obj.AddComponent(typeof(ComplexWaypoint)) as ComplexWaypoint;

        GameObject childObj = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        GameObject.Destroy(childObj.GetComponent<CapsuleCollider>());
        childObj.transform.localEulerAngles = new Vector3(0, 0, 90.0f);
        childObj.transform.parent = obj.transform;
        
        waypoint.SetDroneRequirements(waypoint.flightRadius, waypoint.maxSpeed, DroneOrientationType.TOWARDS_WAYPOINT, waypoint.droneOrientation);

        return waypoint;
    }

    private static Waypoint GenerateComplexWaypoint(float flightRadius, float maxSpeed, DroneOrientationType droneOrientationType, Vector3 droneOrientation) {
        ComplexWaypoint waypoint = GenerateComplexWaypoint() as ComplexWaypoint;
        waypoint.SetDroneRequirements(flightRadius, maxSpeed, droneOrientationType, droneOrientation);

        return waypoint;
    }

}
