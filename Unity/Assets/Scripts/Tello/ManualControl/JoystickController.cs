using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

public class JoystickController : InputDroneController
{
    protected override ControllerState GetNewControllerState() {
        Gamepad pad = Gamepad.current;
        if (pad == null)
        {
            Debug.LogWarning("No gamepad detected!");
            return new ControllerState();
        }

        float lx = pad.leftStick.x.ReadValue();
        float ly = pad.leftStick.y.ReadValue();
        float rx = pad.rightStick.x.ReadValue();
        float ry = pad.rightStick.y.ReadValue();

        return new ControllerState(lx, ly, rx, ry);
    }

    protected override bool IsTakeoffRequested() {
        Gamepad pad = Gamepad.current;
        if (pad == null)
        {
            Debug.LogWarning("No gamepad detected!");
            return false;
        }

        return pad.yButton.wasPressedThisFrame;
    }

    protected override bool IsLandingRequested() {
        Gamepad pad = Gamepad.current;
        if (pad == null)
        {
            Debug.LogWarning("No gamepad detected!");
            return false;
        }

        return pad.aButton.wasPressedThisFrame;
    }
}
