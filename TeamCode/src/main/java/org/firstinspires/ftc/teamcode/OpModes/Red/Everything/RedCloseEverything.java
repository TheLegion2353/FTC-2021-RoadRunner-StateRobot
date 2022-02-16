package org.firstinspires.ftc.teamcode.OpModes.Red.Everything;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Vision;

import java.util.List;

@TeleOp(name="Red Close Everything", group="Autonomous")
public class RedCloseEverything extends LinearOpMode {
	private Robot robot = null;
	private int duckLocation = 2;  // 0: left; 1: middle; 2: right
	private Vision vision = null;

	@Override
	public void runOpMode() throws InterruptedException {
		telemetry.addLine("Initializing autonomous... DO NOT START");
		telemetry.update();

		robot = new Robot(null, telemetry, hardwareMap);
		vision = new Vision(hardwareMap);

		telemetry.addLine("Ready to start.");
		telemetry.update();

		while (!isStarted()) {
			duckLocation = 2;  // by default it is on the right
			double position = vision.update();
			if (position != -1.0) {
				if (position <= 300) {  // left
					duckLocation = 0;
				} else {  // middle
					duckLocation = 1;
				}
			}
			telemetry.addData("Position: ", duckLocation);
			telemetry.update();
		}

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
			switch (duckLocation) {
				case 2: {
					robot.runAuto(Robot.AutonomousPath.RED_CLOSE_EVERYTHING1, this);
				} break;

				case 1: {
					robot.runAuto(Robot.AutonomousPath.RED_CLOSE_EVERYTHING2, this);
				} break;

				case 0: {
					robot.runAuto(Robot.AutonomousPath.RED_CLOSE_EVERYTHING3, this);
				} break;
			}
			requestOpModeStop();
		}
	}
}
