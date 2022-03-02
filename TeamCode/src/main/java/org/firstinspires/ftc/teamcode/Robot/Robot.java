package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

public class Robot {
	public enum AutonomousPath {
		BLUE_CLOSE_EVERYTHING1,
		BLUE_CLOSE_EVERYTHING2,
		BLUE_CLOSE_EVERYTHING3,
		BLUE_CLOSE_PARK_1_TRAJECTORY,
		BLUE_CLOSE_PARK_2_TRAJECTORY,
		BLUE_FAR_PARK_1_TRAJECTORY,
		BLUE_FAR_PARK_2_TRAJECTORY,
		BLUE_CLOSE_CAROUSEL_LEVEL_1_PARK_1_TRAJECTORY,
		BLUE_CLOSE_CAROUSEL_LEVEL_2_PARK_1_TRAJECTORY,
		BLUE_CLOSE_CAROUSEL_LEVEL_3_PARK_1_TRAJECTORY,
		BLUE_CLOSE_CAROUSEL_LEVEL_1_PARK_2_TRAJECTORY,
		BLUE_CLOSE_CAROUSEL_LEVEL_2_PARK_2_TRAJECTORY,
		BLUE_CLOSE_CAROUSEL_LEVEL_3_PARK_2_TRAJECTORY,
		BLUE_FAR_CAROUSEL_LEVEL_1_PARK_1_TRAJECTORY,
		BLUE_FAR_CAROUSEL_LEVEL_2_PARK_1_TRAJECTORY,
		BLUE_FAR_CAROUSEL_LEVEL_3_PARK_1_TRAJECTORY,
		BLUE_FAR_CAROUSEL_LEVEL_1_PARK_2_TRAJECTORY,
		BLUE_FAR_CAROUSEL_LEVEL_2_PARK_2_TRAJECTORY,
		BLUE_FAR_CAROUSEL_LEVEL_3_PARK_2_TRAJECTORY,

		RED_CLOSE_EVERYTHING1,
		RED_CLOSE_EVERYTHING2,
		RED_CLOSE_EVERYTHING3,
		RED_CLOSE_PARK_1_TRAJECTORY,
		RED_CLOSE_PARK_2_TRAJECTORY,
		RED_FAR_PARK_1_TRAJECTORY,
		RED_FAR_PARK_2_TRAJECTORY,
		RED_CLOSE_CAROUSEL_LEVEL_1_PARK_1_TRAJECTORY,
		RED_CLOSE_CAROUSEL_LEVEL_2_PARK_1_TRAJECTORY,
		RED_CLOSE_CAROUSEL_LEVEL_3_PARK_1_TRAJECTORY,
		RED_CLOSE_CAROUSEL_LEVEL_1_PARK_2_TRAJECTORY,
		RED_CLOSE_CAROUSEL_LEVEL_2_PARK_2_TRAJECTORY,
		RED_CLOSE_CAROUSEL_LEVEL_3_PARK_2_TRAJECTORY,
		RED_FAR_CAROUSEL_LEVEL_1_PARK_1_TRAJECTORY,
		RED_FAR_CAROUSEL_LEVEL_2_PARK_1_TRAJECTORY,
		RED_FAR_CAROUSEL_LEVEL_3_PARK_1_TRAJECTORY,
		RED_FAR_CAROUSEL_LEVEL_1_PARK_2_TRAJECTORY,
		RED_FAR_CAROUSEL_LEVEL_2_PARK_2_TRAJECTORY,
		RED_FAR_CAROUSEL_LEVEL_3_PARK_2_TRAJECTORY,
		TEST_TRAJECTORY
	}

	private ElapsedTime clock = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
	private MecanumDrivetrain drivetrain = null;
	private Carousel carousel;
	private Arm arm;
	private Arm wrist;
	private Intake intake;
	private Gamepad gamepad = null;
	private Telemetry telemetry = null;

	private TrajectorySequence BLUE_PARK_INNER = null;
	private TrajectorySequence BLUE_CAROUSEL_LEVEL_1_TRAJECTORY = null;
	private TrajectorySequence BLUE_CAROUSEL_LEVEL_2_TRAJECTORY = null;
	private TrajectorySequence BLUE_CAROUSEL_LEVEL_3_TRAJECTORY = null;
	private TrajectorySequence BLUE_PARK_TRAJECTORY = null;

	private TrajectorySequence RED_PARK_INNER = null;
	private TrajectorySequence RED_CAROUSEL_LEVEL_1_TRAJECTORY = null;
	private TrajectorySequence RED_CAROUSEL_LEVEL_2_TRAJECTORY = null;
	private TrajectorySequence RED_CAROUSEL_LEVEL_3_TRAJECTORY = null;
	private TrajectorySequence RED_PARK_TRAJECTORY = null;

	private TrajectorySequence FINAL_TRAJECTORY = null;

	public Robot(Gamepad gp, Telemetry t, HardwareMap hwMap) {
		telemetry = t;
		gamepad = gp;
		drivetrain = new MecanumDrivetrain(gamepad, telemetry, hwMap);

		setCarouselMotor(hwMap.get(DcMotorEx.class, "spinner"));
		setArm(hwMap.get(DcMotorEx.class, "armMotor"), hwMap.get(AnalogInput.class, "armPot"));
		setWrist(hwMap.get(DcMotorEx.class, "wristMotor"), hwMap.get(AnalogInput.class, "wristPot"));
		wrist.setBounds(0.1, 2.0);
		setIntake(hwMap.servo.get("clawServo"));
	}

	public Robot(Gamepad gp, Telemetry t, HardwareMap hwMap, AutonomousPath path) {
		telemetry = t;
		gamepad = gp;
		drivetrain = new MecanumDrivetrain(gamepad, telemetry, hwMap);

		setCarouselMotor(hwMap.get(DcMotorEx.class, "spinner"));
		setArm(hwMap.get(DcMotorEx.class, "armMotor"), hwMap.get(AnalogInput.class, "armPot"));
		//setWrist(hwMap.get(DcMotorEx.class, "wristMotor"), hwMap.get(AnalogInput.class, "wristPot"));
		//wrist.setBounds(0.65, 2.0);
		setIntake(hwMap.servo.get("clawServo"));

		constructPaths(path);
	}

	public void constructPaths(AutonomousPath path) {
		switch (path) {
			case TEST_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(-36.0, 62.0, -Math.PI / 2.0));
				FINAL_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
						.addTemporalMarker(() -> {
							arm.setPosition(.41);
							wrist.setPosition(1.58);
						})
						.waitSeconds(5)
						.addTemporalMarker(() -> {
							arm.setPosition(2);
							wrist.setPosition(.5);
						})
						.waitSeconds(2)
						.addTemporalMarker(() -> {
							arm.kill();
							wrist.kill();
						})
						.waitSeconds(10)
						.build();
			} break;
			case BLUE_CLOSE_EVERYTHING1: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(-36.0, 62.0, -Math.PI / 2.0));
				FINAL_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
						.lineToConstantHeading(new Vector2d(-14.0, 62.0))  // go in front of the shipping hub
						.addTemporalMarker(() -> {
							//arm.setPosition(.62);
							//wrist.setPosition(1.32);
						})
						.waitSeconds(5)
						.lineToConstantHeading(new Vector2d(-14.0, 48.0))  // go in front of the shipping hub
						.addTemporalMarker(() -> {
							//intake.setPosition(0.0);
						})
						.waitSeconds(0.5)
						.lineToConstantHeading(new Vector2d(-62.5, 57.5))  // go to the carousel thing
						.UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
							carousel.setSpeed(0.5);
						})
						.waitSeconds(3.5)
						.addTemporalMarker(() -> {
							carousel.setSpeed(0.0);
						})
						.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
						.lineToConstantHeading(new Vector2d(-60, 36))  // park
						.addTemporalMarker(() -> {
							//arm.setPosition(2);
							//wrist.setPosition(.5);
						})
						.waitSeconds(2)
						.addTemporalMarker(() -> {
							//	arm.kill();
							//	wrist.kill();
						})
						.resetVelConstraint()
						.build();
			} break;
			case BLUE_CLOSE_EVERYTHING2: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(-36.0, 62.0, -Math.PI / 2.0));
				FINAL_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
						.lineToConstantHeading(new Vector2d(-14.0, 62.0))  // go in front of the shipping hub
						.addTemporalMarker(() -> {
							arm.setPosition(.62);
							wrist.setPosition(1.32);
						})
						.waitSeconds(5)
						.lineToConstantHeading(new Vector2d(-14.0, 48.0))  // go in front of the shipping hub
						.addTemporalMarker(() -> {
							intake.setPosition(0.0);
						})
						.waitSeconds(0.5)
						.lineToConstantHeading(new Vector2d(-62.5, -56.5))  // go to the carousel thing
						.UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
							carousel.setSpeed(0.5);
						})
						.waitSeconds(3.5)
						.addTemporalMarker(() -> {
							carousel.setSpeed(0.0);
						})
						.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
						.lineToConstantHeading(new Vector2d(-60, 34))  // park
						.addTemporalMarker(() -> {
							arm.setPosition(2);
							wrist.setPosition(.5);
						})
						.waitSeconds(2)
						.addTemporalMarker(() -> {
							arm.kill();
							wrist.kill();
						})
						.resetVelConstraint()
						.build();
			} break;
			case BLUE_CLOSE_EVERYTHING3: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(-36.0, 62.0, -Math.PI / 2.0));
				FINAL_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
						.lineToConstantHeading(new Vector2d(-14.0, 55.0))  // go in front of the shipping hub
						.waitSeconds(1.5)
						.addTemporalMarker(() -> {
							arm.setPosition(.46);
							wrist.setPosition(1.58);
						})
						.lineToConstantHeading(new Vector2d(-14.0, 40.0))  // go in front of the shipping hub
						.addTemporalMarker(() -> {
							arm.setPosition(1.715);
						})
						.waitSeconds(1.5)
						.addTemporalMarker(() -> {
							intake.setPosition(0.0);
						})
						.waitSeconds(0.5)
						.addTemporalMarker(() -> {
							arm.setPosition(2.0);
							wrist.setPosition(0.7);
						})
						.waitSeconds(0.5)
						.addTemporalMarker(() -> {
							arm.kill();
							wrist.kill();
						})
						.lineToConstantHeading(new Vector2d(-62.5, 56.5))  // go to the carousel thing
						.UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
							carousel.setSpeed(0.5);
						})
						.waitSeconds(3.5)
						.addTemporalMarker(() -> {
							carousel.setSpeed(0.0);
						})
						.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
						.lineToConstantHeading(new Vector2d(-60, 34))  // park
						.resetVelConstraint()
						.build();
			} break;
			// BLUE LEVEL 1
			case BLUE_CLOSE_CAROUSEL_LEVEL_1_PARK_1_TRAJECTORY: {  // test this first then basically copy for rest of levels
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(-36.0, 62.0, -Math.PI / 2.0));
				FINAL_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
						.lineToConstantHeading(new Vector2d(-14.0, 40.0))  // go in front of the shipping hub
						.addTemporalMarker(() -> {
							arm.setPosition(1.715);
						})
						.waitSeconds(1.5)
						.addTemporalMarker(() -> {
							intake.setSpeed(-0.4);
						})
						.waitSeconds(1.0)
						.addTemporalMarker(() -> {
							intake.setSpeed(0.0);
							arm.setPosition(1.35);
						})
						.lineToConstantHeading(new Vector2d(-62.5, 56.5))  // go to the carousel thing
						.UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
							carousel.setSpeed(1.0);
						})
						.waitSeconds(3.5)
						.addTemporalMarker(() -> {
							carousel.setSpeed(0.0);
						})
						.lineToSplineHeading(new Pose2d(7.5, 64, 0.0))  // set up for going to the parking
						.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
						.lineToConstantHeading(new Vector2d(37, 64.5))  // go through the obstacle
						.resetVelConstraint()
						.build();
			} break;

			case BLUE_FAR_CAROUSEL_LEVEL_1_PARK_1_TRAJECTORY: {  // test this first then basically copy for rest of levels
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(10.0, 62.0, -Math.PI / 2.0));
				FINAL_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
						.lineToConstantHeading(new Vector2d(-14.0, 40.0))  // go in front of the shipping hub
						.addTemporalMarker(() -> {
							arm.setPosition(1.715);
						})
						.waitSeconds(1.5)
						.addTemporalMarker(() -> {
							intake.setSpeed(-0.4);
						})
						.waitSeconds(1.0)
						.addTemporalMarker(() -> {
							intake.setSpeed(0.0);
							arm.setPosition(1.35);
						})
						.lineToConstantHeading(new Vector2d(-62.5, 56.5))  // go to the carousel thing
						.UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
							carousel.setSpeed(1.0);
						})
						.waitSeconds(3.5)
						.addTemporalMarker(() -> {
							carousel.setSpeed(0.0);
						})
						.lineToSplineHeading(new Pose2d(7.5, 64, 0.0))  // set up for going to the parking
						.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
						.lineToConstantHeading(new Vector2d(37, 64.5))  // go through the obstacle
						.resetVelConstraint()
						.build();
			} break;

			case BLUE_CLOSE_CAROUSEL_LEVEL_1_PARK_2_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(-36.0, 62.0, -Math.PI / 2.0));
				FINAL_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
						.lineToConstantHeading(new Vector2d(-14.0, 40.0))  // go in front of the shipping hub
						.addTemporalMarker(() -> {
							arm.setPosition(1.715);
						})
						.waitSeconds(1.5)
						.addTemporalMarker(() -> {
							intake.setSpeed(-0.4);
						})
						.waitSeconds(1.0)
						.addTemporalMarker(() -> {
							intake.setSpeed(0.0);
							arm.setPosition(1.35);
						})
						.lineToConstantHeading(new Vector2d(-62.5, 56.5))  // go to the carousel thing
						.UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
							carousel.setSpeed(1.0);
						})
						.waitSeconds(3.5)
						.addTemporalMarker(() -> {
							carousel.setSpeed(0.0);
						})
						.lineToSplineHeading(new Pose2d(7.5, 64, 0.0))  // set up for going to the parking
						.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
						.lineToConstantHeading(new Vector2d(37, 64.5))  // go through the obstacle
						.resetVelConstraint()
						.strafeRight(23.0)
						.build();
			} break;

			case BLUE_FAR_CAROUSEL_LEVEL_1_PARK_2_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(10.0, 62.0, -Math.PI / 2.0));
				FINAL_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
						.lineToConstantHeading(new Vector2d(-14.0, 40.0))  // go in front of the shipping hub
						.addTemporalMarker(() -> {
							arm.setPosition(1.715);
						})
						.waitSeconds(1.5)
						.addTemporalMarker(() -> {
							intake.setSpeed(-0.4);
						})
						.waitSeconds(1.0)
						.addTemporalMarker(() -> {
							intake.setSpeed(0.0);
							arm.setPosition(1.35);
						})
						.lineToConstantHeading(new Vector2d(-62.5, 56.5))  // go to the carousel thing
						.UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
							carousel.setSpeed(1.0);
						})
						.waitSeconds(3.5)
						.addTemporalMarker(() -> {
							carousel.setSpeed(0.0);
						})
						.lineToSplineHeading(new Pose2d(7.5, 64, 0.0))  // set up for going to the parking
						.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
						.lineToConstantHeading(new Vector2d(37, 64.5))  // go through the obstacle
						.resetVelConstraint()
						.strafeRight(23.0)
						.build();
			} break;

			// BLUE LEVEL 2
			case BLUE_CLOSE_CAROUSEL_LEVEL_2_PARK_1_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(-36.0, 62.0, -Math.PI / 2.0));
			} break;

			case BLUE_FAR_CAROUSEL_LEVEL_2_PARK_1_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(10.0, 62.0, -Math.PI / 2.0));
			} break;

			case BLUE_CLOSE_CAROUSEL_LEVEL_2_PARK_2_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(-36.0, 62.0, -Math.PI / 2.0));
			} break;

			case BLUE_FAR_CAROUSEL_LEVEL_2_PARK_2_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(10.0, 62.0, -Math.PI / 2.0));
			} break;

			// BLUE LEVEL 3
			case BLUE_CLOSE_CAROUSEL_LEVEL_3_PARK_1_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(-36.0, 62.0, -Math.PI / 2.0));
			} break;

			case BLUE_FAR_CAROUSEL_LEVEL_3_PARK_1_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(10.0, 62.0, -Math.PI / 2.0));
			} break;

			case BLUE_CLOSE_CAROUSEL_LEVEL_3_PARK_2_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(-36.0, 62.0, -Math.PI / 2.0));
			} break;

			case BLUE_FAR_CAROUSEL_LEVEL_3_PARK_2_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(10.0, 62.0, -Math.PI / 2.0));
			} break;

			// BLUE PARK ONLY
			case BLUE_CLOSE_PARK_1_TRAJECTORY: {  // deprecated
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(-36.0, 62.0, 0));
				FINAL_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
						.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
						.lineToConstantHeading(new Vector2d(39.0, 62.0))
						.resetVelConstraint()
						.build();
			} break;

			case BLUE_FAR_PARK_1_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(10.0, 62.0, 0));
				FINAL_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
						.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
						.lineToConstantHeading(new Vector2d(39.0, 62.0))
						.resetVelConstraint()
						.build();
			} break;

			case BLUE_CLOSE_PARK_2_TRAJECTORY: {  // deprecated
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(-36.0, 62.0, 0));
				FINAL_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
						.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
						.lineToConstantHeading(new Vector2d(39.0, 62.0))
						.resetVelConstraint()
						.strafeRight(23.0)
						.build();
			} break;

			case BLUE_FAR_PARK_2_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(10.0, 62.0, 0));
				FINAL_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
						.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
						.lineToConstantHeading(new Vector2d(39.0, 62.0))
						.resetVelConstraint()
						.strafeRight(23.0)
						.build();
			} break;

			case RED_CLOSE_EVERYTHING1: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(-36.0, -62.0, Math.PI / 2.0));
				FINAL_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
						.lineToConstantHeading(new Vector2d(-14.0, -62.0))  // go in front of the shipping hub
						.addTemporalMarker(() -> {
							arm.setPosition(.95);
							wrist.setPosition(1.03);
						})
						.waitSeconds(5)
						.lineToConstantHeading(new Vector2d(-14.0, -40.0))  // go in front of the shipping hub
						.addTemporalMarker(() -> {
							intake.setPosition(0.0);
						})
						.waitSeconds(0.5)
						.lineToLinearHeading(new Pose2d(-62.0, -52.5, 0.0))  // go to the carousel thing
						.UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
							carousel.setSpeed(-0.5);
						})
						.waitSeconds(3.5)
						.addTemporalMarker(() -> {
							carousel.setSpeed(0.0);
						})
						.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
						.lineToLinearHeading(new Pose2d(-60, -37, Math.PI / 2.0))  // park
						.addTemporalMarker(() -> {
							arm.setPosition(2);
							wrist.setPosition(.5);
						})
						.waitSeconds(2)
						.addTemporalMarker(() -> {
							arm.kill();
							wrist.kill();
						})
						.resetVelConstraint()
						.build();
			} break;
			case RED_CLOSE_EVERYTHING2: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(-36.0, -62.0, Math.PI / 2.0));
				FINAL_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
						.lineToConstantHeading(new Vector2d(-14.0, -62.0))  // go in front of the shipping hub
						.addTemporalMarker(() -> {
							arm.setPosition(.62);
							wrist.setPosition(1.32);
						})
						.waitSeconds(5)
						.lineToConstantHeading(new Vector2d(-14.0, -48.0))  // go in front of the shipping hub
						.addTemporalMarker(() -> {
							intake.setPosition(0.0);
						})
						.waitSeconds(0.5)
						.lineToLinearHeading(new Pose2d(-62.0, -52.5, 0.0))  // go to the carousel thing
						.UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
							carousel.setSpeed(-0.5);
						})
						.waitSeconds(3.5)
						.addTemporalMarker(() -> {
							carousel.setSpeed(0.0);
						})
						.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
						.lineToLinearHeading(new Pose2d(-60, -37, Math.PI / 2.0))  // park
						.addTemporalMarker(() -> {
							arm.setPosition(2);
							wrist.setPosition(.5);
						})
						.waitSeconds(2)
						.addTemporalMarker(() -> {
							arm.kill();
							wrist.kill();
						})
						.resetVelConstraint()
						.build();
			} break;
			case RED_CLOSE_EVERYTHING3: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(-36.0, -62.0, Math.PI / 2.0));
				FINAL_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
						.lineToConstantHeading(new Vector2d(-14.0, -62.0))  // go in front of the shipping hub
						.addTemporalMarker(() -> {
							arm.setPosition(.41);
							wrist.setPosition(1.58);
						})
						.waitSeconds(5)
						.lineToConstantHeading(new Vector2d(-14.0, -48.0))  // go in front of the shipping hub
						.addTemporalMarker(() -> {
							intake.setPosition(0.0);
						})
						.waitSeconds(0.5)
						.lineToLinearHeading(new Pose2d(-62.0, -52.5, 0.0))  // go to the carousel thing
						.UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
							carousel.setSpeed(-0.5);
						})
						.waitSeconds(3.5)
						.addTemporalMarker(() -> {
							carousel.setSpeed(0.0);
						})
						.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
						.lineToLinearHeading(new Pose2d(-60, -37, Math.PI / 2.0))  // park
						.addTemporalMarker(() -> {
							arm.setPosition(2);
							wrist.setPosition(.5);
						})
						.waitSeconds(2)
						.addTemporalMarker(() -> {
							arm.kill();
							wrist.kill();
						})
						.resetVelConstraint()
						.build();
			} break;

			// RED LEVEL 1
			case RED_CLOSE_CAROUSEL_LEVEL_1_PARK_1_TRAJECTORY: {  // test this first then basically copy for rest of levels
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(-36.0, -62.0, Math.PI / 2.0));
				FINAL_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
						.lineToConstantHeading(new Vector2d(-14.0, -40.0))  // go in front of the shipping hub
						.addTemporalMarker(() -> {
							arm.setPosition(1.715);
						})
						.waitSeconds(1.5)
						.addTemporalMarker(() -> {
							intake.setSpeed(-0.4);
						})
						.waitSeconds(1.0)
						.addTemporalMarker(() -> {
							intake.setSpeed(0.0);
							arm.setPosition(1.35);
						})
						.lineToConstantHeading(new Vector2d(-62.5, -56.5))  // go to the carousel thing
						.UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
							carousel.setSpeed(1.0);
						})
						.waitSeconds(3.5)
						.addTemporalMarker(() -> {
							carousel.setSpeed(0.0);
						})
						.lineToSplineHeading(new Pose2d(7.5, -64, 0.0))  // set up for going to the parking
						.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
						.lineToConstantHeading(new Vector2d(37, -64.5))  // go through the obstacle
						.resetVelConstraint()
						.build();
			} break;

			case RED_FAR_CAROUSEL_LEVEL_1_PARK_1_TRAJECTORY: {  // test this first then basically copy for rest of levels
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(10.0, -62.0, Math.PI / 2.0));
				FINAL_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
						.lineToConstantHeading(new Vector2d(-14.0, -40.0))  // go in front of the shipping hub
						.addTemporalMarker(() -> {
							arm.setPosition(1.715);
						})
						.waitSeconds(1.5)
						.addTemporalMarker(() -> {
							intake.setSpeed(-0.4);
						})
						.waitSeconds(1.0)
						.addTemporalMarker(() -> {
							intake.setSpeed(0.0);
							arm.setPosition(1.35);
						})
						.lineToConstantHeading(new Vector2d(-62.5, -56.5))  // go to the carousel thing
						.UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
							carousel.setSpeed(1.0);
						})
						.waitSeconds(3.5)
						.addTemporalMarker(() -> {
							carousel.setSpeed(0.0);
						})
						.lineToSplineHeading(new Pose2d(7.5, -64, 0.0))  // set up for going to the parking
						.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
						.lineToConstantHeading(new Vector2d(37, -64.5))  // go through the obstacle
						.resetVelConstraint()
						.build();
			} break;

			case RED_CLOSE_CAROUSEL_LEVEL_1_PARK_2_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(-36.0, -62.0, Math.PI / 2.0));
				FINAL_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
						.lineToConstantHeading(new Vector2d(-14.0, -40.0))  // go in front of the shipping hub
						.addTemporalMarker(() -> {
							arm.setPosition(1.715);
						})
						.waitSeconds(1.5)
						.addTemporalMarker(() -> {
							intake.setSpeed(-0.4);
						})
						.waitSeconds(1.0)
						.addTemporalMarker(() -> {
							intake.setSpeed(0.0);
							arm.setPosition(1.35);
						})
						.lineToConstantHeading(new Vector2d(-62.5, -56.5))  // go to the carousel thing
						.UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
							carousel.setSpeed(1.0);
						})
						.waitSeconds(3.5)
						.addTemporalMarker(() -> {
							carousel.setSpeed(0.0);
						})
						.lineToSplineHeading(new Pose2d(7.5, -64, 0.0))  // set up for going to the parking
						.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
						.lineToConstantHeading(new Vector2d(37, -64.5))  // go through the obstacle
						.resetVelConstraint()
						.strafeLeft(23.0)
						.build();
			} break;

			case RED_FAR_CAROUSEL_LEVEL_1_PARK_2_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(10.0, -62.0, Math.PI / 2.0));
				FINAL_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
						.lineToConstantHeading(new Vector2d(-14.0, -40.0))  // go in front of the shipping hub
						.addTemporalMarker(() -> {
							arm.setPosition(1.715);
						})
						.waitSeconds(1.5)
						.addTemporalMarker(() -> {
							intake.setSpeed(-0.4);
						})
						.waitSeconds(1.0)
						.addTemporalMarker(() -> {
							intake.setSpeed(0.0);
							arm.setPosition(1.35);
						})
						.lineToConstantHeading(new Vector2d(-62.5, -56.5))  // go to the carousel thing
						.UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
							carousel.setSpeed(1.0);
						})
						.waitSeconds(3.5)
						.addTemporalMarker(() -> {
							carousel.setSpeed(0.0);
						})
						.lineToSplineHeading(new Pose2d(7.5, -64, 0.0))  // set up for going to the parking
						.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
						.lineToConstantHeading(new Vector2d(37, -64.5))  // go through the obstacle
						.resetVelConstraint()
						.strafeLeft(23.0)
						.build();
			} break;

			// RED LEVEL 2
			case RED_CLOSE_CAROUSEL_LEVEL_2_PARK_1_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(-36.0, -62.0, Math.PI / 2.0));
			} break;

			case RED_FAR_CAROUSEL_LEVEL_2_PARK_1_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(-36.0 + 46.0, -62.0, Math.PI / 2.0));
			} break;

			case RED_CLOSE_CAROUSEL_LEVEL_2_PARK_2_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(-36.0, -62.0, Math.PI / 2.0));
			} break;

			case RED_FAR_CAROUSEL_LEVEL_2_PARK_2_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(-36.0 + 46.0, -62.0, Math.PI / 2.0));
			} break;

			// RED LEVEL 3
			case RED_CLOSE_CAROUSEL_LEVEL_3_PARK_1_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(-36.0, -62.0, Math.PI / 2.0));
			} break;

			case RED_FAR_CAROUSEL_LEVEL_3_PARK_1_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(-36.0 + 46.0, -62.0, Math.PI / 2.0));
			} break;

			case RED_CLOSE_CAROUSEL_LEVEL_3_PARK_2_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(-36.0, -62.0, Math.PI / 2.0));
			} break;

			case RED_FAR_CAROUSEL_LEVEL_3_PARK_2_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(-36.0 + 46.0, -62.0, Math.PI / 2.0));
			} break;

			// RED PARK ONLY
			case RED_CLOSE_PARK_1_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(-36.0, -62.0, 0));
				FINAL_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
						.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
						.lineToConstantHeading(new Vector2d(39.0, -62.0))
						.resetVelConstraint()
						.build();
			} break;

			case RED_FAR_PARK_1_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(10.0, -62.0, 0));
				FINAL_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
						.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
						.lineToConstantHeading(new Vector2d(39.0, -62.0))
						.resetVelConstraint()
						.build();
			} break;

			case RED_CLOSE_PARK_2_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(-36.0, -62.0, 0));
				FINAL_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
						.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
						.lineToConstantHeading(new Vector2d(39.0, -62.0))
						.resetVelConstraint()
						.strafeLeft(23.0)
						.build();
			} break;

			case RED_FAR_PARK_2_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(10.0, -62.0, 0));
				FINAL_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
						.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
						.lineToConstantHeading(new Vector2d(39.0, -62.0))
						.resetVelConstraint()
						.strafeLeft(23.0)
						.build();
			} break;
		}
	}

	public void runAuto(OpMode opMode) throws InterruptedException {
		drivetrain.getDrivetrain().followTrajectorySequence(FINAL_TRAJECTORY);
		opMode.requestOpModeStop();
	}

	public void runAuto(AutonomousPath path, OpMode opMode) throws InterruptedException {
		constructPaths(path);
		drivetrain.getDrivetrain().followTrajectorySequence(FINAL_TRAJECTORY);
		opMode.requestOpModeStop();
	}

	public void autoUpdate() {
		if (arm != null) {
			arm.update();
		} else {
			telemetry.addLine("Intake null!");
		}

		if (wrist != null) {
			wrist.update();
		} else {
			telemetry.addLine("wrist null!");
		}
		telemetry.update();
		clock.reset();
	}

	public void update() {
		if (drivetrain != null) {
			drivetrain.update();
		} else {
			telemetry.addLine("Drivetrain null!");
		}

		if (carousel != null) {
			carousel.update();
		} else {
			telemetry.addLine("Carousel null!");
		}

		if (arm != null) {
			arm.update();
		} else {
			telemetry.addLine("Arm null!");
		}

		if (wrist != null) {
			wrist.update();
		} else {
			telemetry.addLine("Wrist null!");
		}

		if (intake != null) {
			intake.update();
		} else {
			telemetry.addLine("Intake null!");
		}

		telemetry.update();
		clock.reset();
	}

	public void setCarouselMotor(DcMotorEx motor) {
		carousel = new Carousel(gamepad, motor, telemetry);
	}

	public void setArm(DcMotorEx motor, AnalogInput pot) {
		arm = new Arm(gamepad, motor, telemetry);
		arm.setPotentiometer(pot);
	}

	public void setWrist(DcMotorEx motor, AnalogInput pot) {
		wrist = new Wrist(gamepad, motor, telemetry);
		wrist.setPotentiometer(pot);

	}

	public void setIntake(Servo servo) {
		intake = new Intake(gamepad, servo, telemetry);
	}
}