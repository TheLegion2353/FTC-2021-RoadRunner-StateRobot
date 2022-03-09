package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Wrist extends Arm {

	public Wrist(Gamepad gp, DcMotorEx motor, Telemetry tel) {
		super(gp, motor, tel);
		motorController.setDirection(DcMotorSimple.Direction.REVERSE);
		kP = 4.0;
		kI = 3.5;
		kD = 0.3;
		pid = new PID(kP, kI, kD, position);
	}

	public Wrist(Gamepad gp, DcMotorEx motor, Telemetry tel, boolean s) {
		super(gp, motor, tel, s);
		motorController.setDirection(DcMotorSimple.Direction.REVERSE);
		kP = 4.0;
		kI = 3.5;
		kD = 0.3;
		pid = new PID(kP, kI, kD, position);
	}

	@Override
	public void driverUpdate() {
		if (gamepad != null) {
			updatePositions();
			pid.setSetPoint(position);
			double voltage = pid.PIDLoop(motorController.getVoltage());
			motorController.setSpeed(voltage);
			telemetry.addData("Wrist Setpoint: ", position);
			telemetry.addData("Wrist Position: ", motorController.getVoltage());
			telemetry.addData("Wrist Voltage: ", voltage);
			telemetry.addData("Wrist Detent Position: ", positionDetent);
		}
	}

	@Override
	protected void updatePositions() {
		if (manualOverride) {
			if (gamepad.dpad_up) {
				position += 1.0 * pid.getElapsedTime();
			} else if (gamepad.dpad_down) {
				position -= 1.0 * pid.getElapsedTime();
			}

			if (position > endBound) {
				position = endBound;
			}
			if (position < beginBound) {
				position = beginBound;
			}
		} else {
			switch (positionDetent) {
				case 0: {
					if (shared) {
						position = 0.77;
					} else {
						position = 0.36;
					}
				} break;
				case 1: {
					position = 0.655;
				} break;
				default: {
					positionDetent = 0;
				}
			}
		}
	}
}