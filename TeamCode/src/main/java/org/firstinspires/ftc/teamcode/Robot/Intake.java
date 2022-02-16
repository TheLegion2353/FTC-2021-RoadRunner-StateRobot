package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake extends RobotPart {

	private HardwareControllerEx motorController;

	public Intake(Gamepad gp, Servo servo, Telemetry tel) {
		super(gp);
		motorController = new HardwareControllerEx();
		motorController.addServo(servo);
		motorController.setPosition(1.0);
	}

	@Override
	public void driverUpdate() {
		if (gamepad.x) {
			motorController.setPosition(1.0);
		} else if (gamepad.y) {
			motorController.setPosition(0.0);
		}
	}

	void setSpeed(double s) {
		motorController.setSpeed(s);
	}

	void setPosition(double p) { motorController.setPosition(p); }
}