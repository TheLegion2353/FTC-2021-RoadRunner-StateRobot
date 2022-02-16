package org.firstinspires.ftc.teamcode.Robot;
import java.util.ArrayList;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwareController {
	private ArrayList<DcMotor> motors = null;
	private ArrayList<Servo> servos = null;
	private ArrayList<CRServo> crservos = null;
	private DcMotor.Direction direction = DcMotor.Direction.FORWARD;
	private AnalogInput potentiometer = null;

	public HardwareController() {
		motors = new ArrayList<DcMotor>();
		servos = new ArrayList<Servo>();
		crservos = new ArrayList<CRServo>();
	}

	public HardwareController(DcMotor.RunMode mode, DcMotor ... motorArgs) {
		motors = new ArrayList<DcMotor>();
		servos = new ArrayList<Servo>();
		crservos = new ArrayList<CRServo>();
		for (DcMotor mot : motorArgs) {
			mot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			addMotor(mot, mode);
		}
	}

	public void addMotor(DcMotor motor, DcMotor.RunMode mode) {
		motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motor.setMode(mode);
		motor.setDirection(direction);
		motors.add(motor);
	}

	public void addAnalogInput(AnalogInput device) {
		potentiometer = device;
	}

	public void setSpeed(double s) {
		for (DcMotor m : motors) {
			m.setPower(s);
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

	public void resetEncoders(DcMotor.RunMode mode) {
		for (DcMotor m : motors) {
			m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			m.setMode(mode);
		}
	}

	public double getPos() {
		double avrgPos = 0;
		for (DcMotor m : motors) {
			avrgPos += m.getCurrentPosition();
		}
		avrgPos /= (double)motors.size();
		return avrgPos;
	}

	public double getVoltage() {
		return potentiometer.getVoltage();
	}
	public void addMotor(DcMotor motor) {
		motors.add(motor);
	}

	public void addServo(Servo servo) {
		servos.add(servo);
	}

	public void addCRServo(CRServo crservo) {
		crservos.add(crservo);
	}

	public void setDirection(DcMotor.Direction dir) {
		for (DcMotor m : motors) {
			m.setDirection(dir);
		}
	}

	public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior b) {
		for (DcMotor m : motors) {
			m.setZeroPowerBehavior(b);
		}
	}
}