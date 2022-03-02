package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Wrist extends Arm {

	public Wrist(Gamepad gp, DcMotorEx motor, Telemetry tel) {
		super(gp, motor, tel);
		motorController.setDirection(DcMotorSimple.Direction.REVERSE);
		kP = 3.0;
		kI = 1.0;
		kD = 0.0;
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
				position += 1.5 * pid.getElapsedTime();
			} else if (gamepad.dpad_down) {
				position -= 1.5 * pid.getElapsedTime();
			}

			if (position > endBound) {
				position = endBound;
			}
			if (position < beginBound) {
				position = beginBound;
			}
		} else {
			switch (positionDetent) {
				case 0: {  // back
					position = 1.56;
				}
				break;

				case 1: {  // down
					position = 1.58;
				}
				break;

				case 2: {  // lvl 1
					position = 1.445;
				}
				break;

				case 3: {  // lvl 2
					position = 1.264;
				}
				break;

				case 4: {  // lvl 3
					position = 1.03;
				}
				break;

				default: {
					positionDetent = 0;
				}
			}
		}
	}
}