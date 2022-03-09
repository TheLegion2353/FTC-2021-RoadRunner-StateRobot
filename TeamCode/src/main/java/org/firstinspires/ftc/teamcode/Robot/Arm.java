package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.checkerframework.checker.index.qual.LTEqLengthOf;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm extends RobotPart {
	protected double kP = 7.5;
	protected double kI = 0.25;
	protected double kD = 0.25;
	protected double position = 0.0;
	protected static int positionDetent = 0;
	protected Telemetry telemetry = null;

	protected HardwareControllerEx motorController;
	protected PID pid;
	protected double beginBound, endBound;

	protected static boolean rightBumper, leftBumper;
	protected static boolean manualOverride = true;
	protected boolean kill = false;
	protected boolean shared = false;

	public Arm(Gamepad gp, DcMotorEx motor, Telemetry tel) {
		super(gp);
		telemetry = tel;
		position = 0.0;
		pid = new PID(kP, kI, kD, position);
		pid.setErrorOverTimeMax(0.15);
		motorController = new HardwareControllerEx(tel, DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, null, motor);
		motorController.setDirection(DcMotorSimple.Direction.REVERSE);
		beginBound = 0.67;
		endBound = 1.739;
		leftBumper = false;
		rightBumper = false;
		manualOverride = true;
	}

	public Arm(Gamepad gp, DcMotorEx motor, Telemetry tel, boolean s) {
		super(gp);
		shared = s;
		telemetry = tel;
		position = 0.0;
		pid = new PID(kP, kI, kD, position);
		pid.setErrorOverTimeMax(0.15);
		motorController = new HardwareControllerEx(tel, DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, null, motor);
		motorController.setDirection(DcMotorSimple.Direction.REVERSE);
		beginBound = 0.67;
		endBound = 1.739;
		leftBumper = false;
		rightBumper = false;
		manualOverride = true;
	}

	@Override
	public void driverUpdate() {
		if (gamepad != null) {
			if (gamepad.right_bumper) {
				if (!rightBumper) {
					incrimentDetent();
					rightBumper = true;
				}
			} else {
				rightBumper = false;
			}

			if (gamepad.left_bumper) {
				if (!leftBumper) {
					decrementDetent();
					leftBumper = true;
				}
			} else {
				leftBumper = false;
			}

			if (gamepad.start) {
				manualOverride = false;
			}

			if (gamepad.back) {
				manualOverride = true;
			}

			updatePositions();
			pid.setSetPoint(position);
			double voltage = pid.PIDLoop(motorController.getVoltage());
			motorController.setSpeed(voltage);
			telemetry.addData("Arm Setpoint: ", position);
			telemetry.addData("Arm Position: ", motorController.getVoltage());
			telemetry.addData("Arm Voltage: ", voltage);
			telemetry.addData("Arm Detent Position: ", positionDetent);
		}
	}

	@Override
	protected void autonomousUpdate() {
		if (position > endBound) {
			position = endBound;
		}
		if (position < beginBound) {
			position = beginBound;
		}
		pid.setSetPoint(position);
		double voltage = pid.PIDLoop(motorController.getVoltage());
		if (!kill) {
			motorController.setSpeed(voltage);
		} else {
			motorController.setSpeed(0.0);
		}
		telemetry.addData("Setpoint: ", position);
		telemetry.addData("Position: ", motorController.getVoltage());
	}

	protected void updatePositions() {
		if (manualOverride) {
			position += (gamepad.right_trigger - gamepad.left_trigger) * .75 * pid.getElapsedTime();
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
						position = beginBound;
					} else {
						position = 1.2;
					}
				} break;
				case 1: {
					position = endBound;
				} break;
				default: {
					positionDetent = 0;
				}
			}
		}
	}

	protected void incrimentDetent() {
		pid.resetErrorOverTime();
		positionDetent++;
		if (positionDetent > 1) {
			positionDetent = 0;
		}
	}

	protected void decrementDetent() {
		pid.resetErrorOverTime();
		positionDetent--;
		if (positionDetent < 0) {
			positionDetent = 1;
		}
	}

	public void kill() {
		kill = !kill;
	}

	public void setPosition(double p) {
		pid.resetErrorOverTime();
		position = p;
	}

	public void setPotentiometer(AnalogInput pot) {
		motorController.addAnalogInput(pot);
		position = pot.getVoltage();
	}

	public void setBounds(double begin, double end) {
		beginBound = begin;
		endBound = end;
	}

	public PID getPID() {
		return pid;
	}
}