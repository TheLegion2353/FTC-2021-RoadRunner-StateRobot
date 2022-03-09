package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp(name="Main Mecanum RR Shared", group="Driver Controlled")
public class MainTeleOp2 extends OpMode {
	private Robot robot = null;

	@Override
	public void init() {
		robot = new Robot(gamepad1, telemetry, hardwareMap, true);
	}

	@Override
	public void loop() {
		robot.update();
	}
}
