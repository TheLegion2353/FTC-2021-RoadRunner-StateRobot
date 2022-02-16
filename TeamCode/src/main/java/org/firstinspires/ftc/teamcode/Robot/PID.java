package org.firstinspires.ftc.teamcode.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

public class PID {
	public double kP = 0;
	public double kI = 0;
	public double kD = 0;
	protected double eTime = 0.0;
	protected Telemetry telemetry;
	protected double errorOverTimeMax = 10;
	protected double setPoint = 0;
	protected double error = 0;
	protected double previousError = 0;
	protected double errorOverTime = 0;
	protected ElapsedTime clock = null;

	public PID(double P, double I, double D, double sp) { //first 3 arguments are the p, i, and d constants for a PID.  They need to be experimentally determined.
        /*
        TUNING INSTRUCTIONS:
        The PID values work as such.
        The P value will give a force/voltage proportional to the distance from the set point the current position.
        The I value will give a force/voltage for accumulated errors over time, usually small errors.  If error is small, this I value should prevent that.
        The D value will give a force/voltage to prevent overshooting.  If the set point is far away from the current position, it may overshoot, this D value prevents this.
        In terms of calculus, the P value is the proportional value, the I value is the integral, and the D value is the derivative.
        The order in which the PID should be tuned is the P value, the D value, then lastly, the I value.
        For all the constants, you need to change orders of magnitude initially then change the number itself.
        Ex. 1 -> 0.1 -> 0.01 -> 0.05 -> 0.035
        For tuning the P value, a good value will be able to hold its position with minimal oscillation.  It does not necessarily need to be at the set point, but it needs to be as close as possible without oscillation.
        For tuning the D value, a good value will be able to dampen the movement of the controller to prevent overshooting.  It doesn't need to be as exact as the P value, but it does need to prevent the controller from overshooting.
        For tuning the I value, a good value will be able to allow the controller to move according to small errors.  It needs to be low enough to prevent overshooting, but high enough to let the controller adjust in a timely manner.
         */
		kP = P;
		kI = I;
		kD = D;
		setPoint = sp;
		error = setPoint - error;
		previousError = error;
		clock = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
	}

	public PID(double P, double I, double D, double sp, Telemetry telemetry) { //first 3 arguments are the p, i, and d constants for a PID.  They need to be experimentally determined.
        /*
        TUNING INSTRUCTIONS:
        The PID values work as such.
        The P value will give a force/voltage proportional to the distance from the set point the current position.
        The I value will give a force/voltage for accumulated errors over time, usually small errors.  If error is small, this I value should prevent that.
        The D value will give a force/voltage to prevent overshooting.  If the set point is far away from the current position, it may overshoot, this D value prevents this.
        In terms of calculus, the P value is the proportional value, the I value is the integral, and the D value is the derivative.
        The order in which the PID should be tuned is the P value, the D value, then lastly, the I value.
        For all the constants, you need to change orders of magnitude initially then change the number itself.
        Ex. 1 -> 0.1 -> 0.01 -> 0.05 -> 0.035
        For tuning the P value, a good value will be able to hold its position with minimal oscillation.  It does not necessarily need to be at the set point, but it needs to be as close as possible without oscillation.
        For tuning the D value, a good value will be able to dampen the movement of the controller to prevent overshooting.  It doesn't need to be as exact as the P value, but it does need to prevent the controller from overshooting.
        For tuning the I value, a good value will be able to allow the controller to move according to small errors.  It needs to be low enough to prevent overshooting, but high enough to let the controller adjust in a timely manner.
         */
		kP = P;
		kI = I;
		kD = D;
		setPoint = sp;
		this.telemetry = telemetry;
		error = setPoint - error;
		previousError = error;
		clock = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
	}

	public double PIDLoop(double currentPos) {
		eTime = (double)clock.time(TimeUnit.MILLISECONDS) / 1000.0;
		clock.reset();
		return PIDLoopInternal(currentPos, eTime);
	}

	protected double PIDLoopInternal(double currentVal, double elapsedTime) {
		double processVar = 0;
		calcErrors(currentVal, elapsedTime);
		processVar += calcP() + calcI() + calcD(elapsedTime);
		previousError = error;
		if (telemetry != null) { telemetry.addData("Process Var: ", processVar); }
		return processVar;  // process variable is the voltage to be given to the motor or component.
	}

	public void setSetPoint(double sp) {  // call this function in order to set the set point.
		setPoint = sp;
	}

	public void updateConst(double P, double I, double D) {  // update the PID constants if need be.  Usually won't need to be called.
		kP = P;
		kI = I;
		kD = D;
	}

	public double getElapsedTime() {
		return eTime;
	}

	public void setErrorOverTimeMax(double max) {
		errorOverTimeMax = max;
	}

	public void resetErrorOverTime() {
		errorOverTime = 0.0;
	}
	// these private functions are only used internally to calculate the different values.
	protected double calcP() {
		return (error * kP);
	}

	protected double calcI() {
		return (double)errorOverTime * (double)kI;
	}

	protected double calcD(double elapsedTime) {
		return ((double)(error - previousError)/(double)elapsedTime) * (double)kD;
	}

	protected void calcErrors(double currentVal, double elapsedTime) {
		error = setPoint - currentVal;
		errorOverTime += error * elapsedTime;
		if (errorOverTime > errorOverTimeMax) {
			errorOverTime = errorOverTimeMax;
		} else if (errorOverTime < -errorOverTimeMax) {
			errorOverTime = -errorOverTimeMax;
		}
	}
}