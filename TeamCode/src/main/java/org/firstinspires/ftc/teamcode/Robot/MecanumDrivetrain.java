package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

public class MecanumDrivetrain extends RobotPart {

	private Telemetry telemetry;
	private SampleMecanumDrive mecanum;
	private PIDFController headingController;
	private ElapsedTime clock;
	private double theta;
	private int state = 0;

	public MecanumDrivetrain(Gamepad gp, Telemetry tel, HardwareMap hwMap) {
		super(gp);
		telemetry = tel;
		mecanum = new SampleMecanumDrive(hwMap);
		mecanum.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);
		headingController.setInputBounds(-Math.PI, Math.PI);
		clock = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
	}

	public void followTrajectorySequence(TrajectorySequence sequence) {
		mecanum.followTrajectorySequence(sequence);
	}

	public void followTrajectory(Trajectory trajectory) {
		mecanum.followTrajectory(trajectory);
	}

	public void setPoseEstimate(Pose2d pose) {
		mecanum.setPoseEstimate(pose);
	}
	@Override
	public void driverUpdate() {
		Pose2d driveDirection = new Pose2d();
		Pose2d poseEstimate = mecanum.getPoseEstimate();
		telemetry.addData("x", poseEstimate.getX());
		telemetry.addData("y", poseEstimate.getY());
		telemetry.addData("heading", poseEstimate.getHeading());

		if (gamepad != null) {
			theta +=  3.0 * -gamepad.right_stick_x * (double)clock.time(TimeUnit.MILLISECONDS) / 1000.0;
			if (theta - poseEstimate.getHeading() > 0.25) {
				theta = poseEstimate.getHeading() + 0.25;
			}

			if (theta - poseEstimate.getHeading() < -0.25) {
				theta = poseEstimate.getHeading() - 0.25;
			}

			if (gamepad.left_stick_x == 0.0 && gamepad.left_stick_y == 0.0 && gamepad.right_stick_x == 0.0) {
				theta = poseEstimate.getHeading();
			}
			clock.reset();
			headingController.setTargetPosition(theta);

			double headingInput = (headingController.update(poseEstimate.getHeading()) * DriveConstants.kV) * DriveConstants.TRACK_WIDTH;
			driveDirection = new Pose2d(
					-gamepad.left_stick_y,
					-gamepad.left_stick_x,
					headingInput
			);

			mecanum.setWeightedDrivePower(driveDirection);
			headingController.update(poseEstimate.getHeading());
		}
		telemetry.addData("theta", theta);
		mecanum.update();
	}

	@Override
	protected void autonomousUpdate() {
		switch (state) {
			default: {

			}
		}
	}

	public SampleMecanumDrive getDrivetrain() {
		return mecanum;
	}
}