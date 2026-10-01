using System;
using UnityEngine;

public abstract class Waypoint : MonoBehaviour
{
    public enum Type {
        SIMPLE,
        COMPLEX
    };

    protected event Action<object, WaypointReachedEvent> OnWaypointReached;
    protected event Action<object, WaypointReachedEvent> OnWaypointHit;
    public class WaypointReachedEvent : EventArgs
    {
        public WaypointReachedEvent(Collider other)
        {
            this.other = other;
        }

        public Collider other { get; }
    }

    public enum WaypointState {
        DEFAULT,
        ACTIVE,
        COMPLETED
    }

    protected WaypointState state;

    private static Material colorWhite;
    private static Material colorGreen;
    private static Material colorRed;
    private int index;

    public int GetIndex()
    {
        return index;
    }

    public void SetIndex(int index)
    {
        this.index = index;
    }

    public abstract void SetTriggerRadius(float triggerRadius);

    public virtual void ResetWaypoint()
    {
        state = WaypointState.DEFAULT;
        UpdateMaterial();
    }

    public void SetTransformData(Parcours parent, Vector3 position, Quaternion orientation) {
        transform.parent = parent.transform;
        transform.localPosition = position;
        transform.rotation = orientation;
    }

    public void SetWaypointReachedCallback(Action<object, WaypointReachedEvent> waypointCallback) {
        OnWaypointReached = waypointCallback;
    }

    public void SetWaypointHitCallback(Action<object, WaypointReachedEvent> waypointCallback) {
        OnWaypointHit = waypointCallback;
    }

    public virtual void SetActiveWaypoint()
    {
        state = WaypointState.ACTIVE;
        UpdateMaterial();
        OnWaypointActivated();
    }

    public virtual void OnWaypointActivated(){}

    protected void CallOnWaypointHit(WaypointReachedEvent waypointHitEvent)
    {
        OnWaypointHit?.Invoke(this, waypointHitEvent);
    }

    protected void CallOnWaypointReached(WaypointReachedEvent waypointReachedEvent)
    {
        OnWaypointReached?.Invoke(this, waypointReachedEvent);
    }

    protected void UpdateMaterial() {
        switch (state) {
            case WaypointState.ACTIVE:
                if (colorGreen == null)
                {
                    colorGreen = new Material(Shader.Find("Universal Render Pipeline/Lit"));
                    colorGreen.SetFloat("_Smoothness", 0f);
                    colorGreen.color = Color.green;
                }
                
                transform.GetChild(0).GetComponent<MeshRenderer>().sharedMaterial = colorGreen;
                break;
            case WaypointState.COMPLETED:
                if (colorRed == null)
                {
                    colorRed = new Material(Shader.Find("Universal Render Pipeline/Lit"));
                    colorRed.SetFloat("_Smoothness", 0f);
                    colorRed.color = Color.red;
                }
                
                transform.GetChild(0).GetComponent<MeshRenderer>().sharedMaterial = colorRed;
                break;
            default:
                if (colorWhite == null)
                {
                    colorWhite = new Material(Shader.Find("Universal Render Pipeline/Lit"));
                    colorWhite.SetFloat("_Smoothness", 0f);
                    colorWhite.color = new Color(0.8f, 0.8f, 0.8f);
                }

                transform.GetChild(0).GetComponent<MeshRenderer>().sharedMaterial = colorWhite;
                break;
        }
    }
}
