using UnityEngine;

public class ControllerState
{
    public float lx, ly, rx, ry;
    public float speed; // TODO: Seems to be always treated as 0
    public float deadBand = 0.15f;

    public ControllerState() : this(0, 0, 0, 0) {}

    public ControllerState(float lx, float ly, float rx, float ry) : this(lx, ly, rx, ry, 1) {}

    public ControllerState(float lx, float ly, float rx, float ry, float speed)
    {
        setAxis(lx, ly, rx, ry);
        setSpeedMode(speed);
    }

    public void setAxis(float lx, float ly, float rx, float ry)
    {
        this.lx = lx;
        this.ly = ly;
        this.rx = rx;
        this.ry = ry;
    }
    public void setSpeedMode(float mode)
    {
        speed = mode;
    }

    public override bool Equals(object obj)
    {
        ControllerState otherState = obj as ControllerState;
        return Mathf.Approximately(lx, otherState.lx)
            && Mathf.Approximately(ly, otherState.ly)
            && Mathf.Approximately(rx, otherState.rx)
            && Mathf.Approximately(ry, otherState.ry)
            && Mathf.Approximately(speed, otherState.speed)
            && Mathf.Approximately(deadBand, otherState.deadBand);
    }

    public override int GetHashCode()
    {
        return base.GetHashCode();
    }

    public override string ToString()
    {
        return "[LX: " + lx + " LY: " + ly + " RX: " + rx + " RY: " + ry + "]";
    }
}