using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class KeyboardController : InputDroneController
{
	protected override ControllerState GetNewControllerState() {
		float lx = 0f;
		float ly = 0f;
		float rx = 0f;
		float ry = 0f;

		// Forward
		if (Input.GetKey(KeyCode.UpArrow))
		{
			ry += 1;
		}
		// Backward
		if (Input.GetKey(KeyCode.DownArrow))
		{
			ry += -1;
		}
		// Right
		if (Input.GetKey(KeyCode.RightArrow))
		{
			rx += 1;
		}
		// Left
		if (Input.GetKey(KeyCode.LeftArrow))
		{
			rx += -1;
		}
		// Up
		if (Input.GetKey(KeyCode.W))
		{
			ly += 1;
		}
		// Down
		if (Input.GetKey(KeyCode.S))
		{
			ly += -1;
		}
		// Rotate right
		if (Input.GetKey(KeyCode.D))
		{
			lx += 1;
		}
		// Rotate left
		if (Input.GetKey(KeyCode.A))
		{
			lx += -1;
		}

		return new ControllerState(lx, ly, rx, ry);
	}

	protected override bool IsTakeoffRequested() {
		return Input.GetKeyDown(KeyCode.T);
	}

	protected override bool IsLandingRequested() {
		return Input.GetKeyDown(KeyCode.L);
	}
}
