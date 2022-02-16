package org.firstinspires.ftc.teamcode.Robot;
import java.util.ArrayList;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class HardwareControllerEx {
	private Telemetry telemetry = null;
	private ArrayList<PID> pids = null;
	private ArrayList<DcMotorEx> motors = null;
	private ArrayList<Servo> servos = null;
	private ArrayList<CRServo> crservos = null;
	private DcMotorEx.Direction direction = DcMotorEx.Direction.FORWARD;
	private AnalogInput potentiometer = null;

	public HardwareControllerEx() {
		pids = new ArrayList<PID>();
		motors = new ArrayList<DcMotorEx>();
		servos = new ArrayList<Servo>();
		crservos = new ArrayList<CRServo>();
	}

	public HardwareControllerEx(DcMotorEx.RunMode mode, PID pid, DcMotorEx ... motorArgs) {
		pids = new ArrayList<PID>();
		motors = new ArrayList<DcMotorEx>();
		servos = new ArrayList<Servo>();
		crservos = new ArrayList<CRServo>();
		for (DcMotorEx mot : motorArgs) {
			mot.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
			addMotor(mot, mode, pid);
		}
	}

	public HardwareControllerEx(Telemetry telemetry, DcMotorEx.RunMode mode, PID pid, DcMotorEx ... motorArgs) {
		this.telemetry = telemetry;
		pids = new ArrayList<PID>();
		motors = new ArrayList<DcMotorEx>();
		servos = new ArrayList<Servo>();
		crservos = new ArrayList<CRServo>();
		for (DcMotorEx mot : motorArgs) {
			mot.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
			addMotor(mot, mode, pid);
		}
	}

	public void addMotor(DcMotorEx motor, DcMotorEx.RunMode mode) {
		motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		motor.setMode(mode);
		motor.setDirection(direction);
		motors.add(motor);
		pids.add(null);
	}

	public void addMotor(DcMotorEx motor, DcMotorEx.RunMode mode, PID pid) {
		motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		motor.setMode(mode);
		motor.setDirection(direction);
		motors.add(motor);
		pids.add(pid);
	}

	public void addAnalogInput(AnalogInput device) {
		potentiometer = device;
	}

	public void setSpeed(double s) {
		for (int i = 0; i < motors.size(); i++) {
			if (pids.get(i) != null) {
				pids.get(i).setSetPoint(s);
				motors.get(i).setPower(pids.get(i).PIDLoop(motors.get(i).getCurrentPosition()));
			} else {
				motors.get(i).setPower(s);
			}
		}

		for (CRServo cr : crservos) {
			cr.setPower(s);
		}
	}

	public void setPosition(double p) {
		for (Servo s : servos) {
			s.setPosition(p);
		}
	}

	public void resetEncoders(DcMotorEx.RunMode mode) {
		for (DcMotorEx m : motors) {
			m.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
			m.setMode(mode);
		}
	}

	public double getPos() {
		double avrgPos = 0;
		for (DcMotorEx m : motors) {
			avrgPos += m.getCurrentPosition();
		}
		avrgPos /= (double)motors.size();
		return avrgPos;
	}

	public double getVoltage() {
		return potentiometer.getVoltage();
	}

	public void addMotor(DcMotorEx motor) {
		motors.add(motor);
		pids.add(null);
	}

	public void addMotor(DcMotorEx motor, PID pid) {
		motors.add(motor);
		pids.add(pid);
	}

	public void addServo(Servo servo) {
		servos.add(servo);
	}

	public void addCRServo(CRServo crservo) {
		crservos.add(crservo);
	}

	public void setDirection(DcMotorEx.Direction dir) {
		for (DcMotorEx m : motors) {
			m.setDirection(dir);
		}
	}

	public void setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior b) {
		for (DcMotorEx m : motors) {
			m.setZeroPowerBehavior(b);
		}
	}

	public PID getPID(int index) {
		return pids.get(index);
	}
	public double getVelocity() {
		double avrgVel = 0;
		for (DcMotorEx m : motors) {
			avrgVel += m.getVelocity(AngleUnit.RADIANS);
		}
		avrgVel /= (double)motors.size();
		return avrgVel;
	}
}