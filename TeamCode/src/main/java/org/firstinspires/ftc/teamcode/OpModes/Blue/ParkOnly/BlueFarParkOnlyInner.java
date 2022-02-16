package org.firstinspires.ftc.teamcode.OpModes.Blue.ParkOnly;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp(name="Blue Far Park Only Inner", group="Autonomous")
public class BlueFarParkOnlyInner extends LinearOpMode {
	private Robot robot = null;

	@Override
	public void runOpMode() throws InterruptedException {
		telemetry.addData(">", "Initializing autonomous... DO NOT START");
		telemetry.update();
		robot = new Robot(null, telemetry, hardwareMap, Robot.AutonomousPath.BLUE_FAR_PARK_2_TRAJECTORY);
		//robot.setCarouselMotor(hardwareMap.get(DcMotorEx.class, "carousel/frontEncoder"));
		//robot.setArm(hardwareMap.get(DcMotorEx.class, "arm/leftEncoder"), hardwareMap.get(AnalogInput.class, "armPot"));
		//robot.setLinearSlide(hardwareMap.get(DcMotorEx.class, "slide"));
		//robot.setIntake(hardwareMap.get(DcMotorEx.class, "intake/rightEncoder"));
		waitForStart();
		Thread thread = new Thread() {
			public void run() {
				while (!isStopRequested()) {
					robot.autoUpdate();
				}
			}
		};
		thread.start();

		if (!isStopRequested()) {
			robot.runAuto(this);
			requestOpModeStop();
		}
	}
}
