package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.Gamepad;

public class RobotPart {
	public Gamepad gamepad = null;
	public RobotPart(Gamepad gp) {
		gamepad = gp;
	}

	public void update() {
		if (gamepad != null) {
			driverUpdate();
		} else {
			autonomousUpdate();
		}
	}

	protected void autonomousUpdate() {

	}

	protected void driverUpdate() {

	}

	public void setGamepad(Gamepad gp) {
		gamepad = gp;
	}
}