package org.firstinspires.ftc.teamcode.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PID1stDerivative extends PID {

	protected double lastPos = 0.0;
	protected double currentVel = 0.0;

	public PID1stDerivative(double P, double I, double D, double sp) {
		super(P, I, D, sp);
	}

	public PID1stDerivative(double P, double I, double D, double sp, Telemetry telemetry) {
		super(P, I, D, sp, telemetry);
	}

	protected double getVelocity(double currentVal, double elapsedTime) {
		currentVel = (currentVal - lastPos) / elapsedTime;
		return currentVel;
	}

	@Override
	protected double PIDLoopInternal(double currentVal, double elapsedTime) {

		telemetry.addData("Velocity: ", getVelocity(currentVal, elapsedTime));
		telemetry.addData("Set Point: ", setPoint);
		double returnVal = super.PIDLoopInternal(getVelocity(currentVal, elapsedTime), elapsedTime);
		lastPos = currentVal;
		return returnVal;
	}
}