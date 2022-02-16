package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.apache.commons.math3.analysis.integration.IterativeLegendreGaussIntegrator;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LinearSlide extends RobotPart {
	private double kP = 0.01;
	private double kI = 0.01;
	private double kD = 0.0;
	private double position = 0.0;
	private Telemetry telemetry;
	private final double topMAX = 4000.0;

	private HardwareControllerEx motorController;

	public LinearSlide(Gamepad gp, DcMotorEx motor, Telemetry tel) {
		super(gp);
		telemetry = tel;
		position = 0.0;
		motorController = new HardwareControllerEx(tel, DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, new PID(kP, kI, kD, position), motor);
	}

	@Override
	public void driverUpdate() {
		if (gamepad.dpad_up) {
			position -= 500.0 * motorController.getPID(0).getElapsedTime();
		} else if (gamepad.dpad_down) {
			position += 500.0 * motorController.getPID(0).getElapsedTime();
		}

		if (0.0 < position) {
			position = 0.0;
		}

		if (position <= -topMAX) {
			position = -topMAX;
		}

		motorController.setSpeed(position);
		telemetry.addData("Set Point of Linear Slide: ", position);
		telemetry.addData("Position of Linear Slide: ", motorController.getPos());
	}

	@Override
	protected void autonomousUpdate() {
		motorController.setSpeed(position);
	}

	public void setPosition(double p) {
		position = p;
	}
}