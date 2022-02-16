package org.firstinspires.ftc.teamcode.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MotorPID extends PID1stDerivative {

	public MotorPID(double P, double I, double D, double sp) {
		super(P, I, D, sp);
	}

	public MotorPID(double P, double I, double D, double sp, Telemetry telemetry) {
		super(P, I, D, sp, telemetry);
	}

	@Override
	protected double PIDLoopInternal(double currentVal, double elapsedTime) {
		double velocity = getVelocity(currentVal, elapsedTime);
		telemetry.addData("Velocity 2: ", velocity);

		double returnVal = super.PIDLoopInternal(currentVal, elapsedTime);

		if (Math.abs(setPoint) > Math.abs(velocity)) {  // |setpoint| > |velocity|, meaning you need to speed up
			return returnVal;
		} else {  // you need to slow down, don't apply an opposite voltage.
			return 0.0;
		}

	}
}