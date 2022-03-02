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

import java.util.List;

@Disabled
@TeleOp(name="Red Far Everything Outer", group="Autonomous")
public class RedFarEverythingOuter extends LinearOpMode {
	private Robot robot = null;
	private int duckLocation = 0;  // 0: left; 1: middle; 2: right

	// TENSOR FLOW RELATED STUFF BELOW
	/* Note: This sample uses the all-objects Tensor Flow model (FreightFrenzy_BCDM.tflite), which contains
	 * the following 4 detectable objects
	 *  0: Ball,
	 *  1: Cube,
	 *  2: Duck,
	 *  3: Marker (duck location tape marker)
	 *
	 *  Two additional model assets are available which only contain a subset of the objects:
	 *  FreightFrenzy_BC.tflite  0: Ball,  1: Cube
	 *  FreightFrenzy_DM.tflite  0: Duck,  1: Marker
	 */

	private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
	private static final String[] LABELS = {
			"Ball",
			"Cube",
			"Duck",
			"Marker"
	};
	private static final String VUFORIA_KEY =
			"Aecx07L/////AAABmQbDimTWOUPdkStEP3xpsklJgTSeNK1GUg1sse6qFp4arinGemTI6WwY5YGIKzR5yXW7hzwB+4aLFEDVBIz7EsMvWbH3LG4FeTLS7HiSFFrC1gtOx31tlNZTxNtr9hoOBKi0NAgaQKlMLGGz2xj4Dnw8uxUKZPh7/V9s5NRI2n1LZIGczBGMWJB3UO0wmjk3wKsuFxl519fpP7C53g1z9d9f74KyAGhDNrEXUELNIYHgaAPjDj2BMHpwFxJh0om1l90Hx3/pq6dbh6LFAPHpLVtbBkB9zPLfdIPqTwWEuim00LDY4gT4eHZfvIMD8G5BoigjC8KS9/kIJ5TklVUE4orSsl15oPqJv1t3tDRYZRDu";

	/**
	 * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
	 * localization engine.
	 */
	private VuforiaLocalizer vuforia;

	/**
	 * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
	 * Detection engine.
	 */
	private TFObjectDetector tfod;
	// TENSOR FLOW STUFF ABOVE

	@Override
	public void runOpMode() throws InterruptedException {
		telemetry.addData(">", "Initializing autonomous... DO NOT START");
		telemetry.update();
		robot = new Robot(null, telemetry, hardwareMap);
		//robot.setCarouselMotor(hardwareMap.get(DcMotorEx.class, "carousel/frontEncoder"));
		//robot.setArm(hardwareMap.get(DcMotorEx.class, "arm/leftEncoder"), hardwareMap.get(AnalogInput.class, "armPot"));
		//robot.setWrist(hardwareMap.get(DcMotorEx.class, "wrist"));
		//robot.setIntake(hardwareMap.get(DcMotorEx.class, "intake/rightEncoder"));
		initVuforia();
		initTfod();

		if (tfod != null) {
			tfod.activate();

			// The TensorFlow software will scale the input images from the camera to a lower resolution.
			// This can result in lower detection accuracy at longer distances (> 55cm or 22").
			// If your target is at distance greater than 50 cm (20") you can adjust the magnification value
			// to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
			// should be set to the value of the images used to create the TensorFlow Object Detection model
			// (typically 16/9).
			tfod.setZoom(1.0, 16.0/9.0);
		}

		while (!isStarted()) {
			if (tfod != null) {
				// getUpdatedRecognitions() will return null if no new information is available since
				// the last time that call was made.
				List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
				if (updatedRecognitions != null) {
					telemetry.addData("# Object Detected", updatedRecognitions.size());
					// step through the list of recognitions and display boundary info.
					int i = 0;
					duckLocation = 0;
					for (Recognition recognition : updatedRecognitions) {
						if (recognition.getLabel().equals("Duck")) {
							if (recognition.getLeft() > 200.0 && recognition.getLeft() < 300.0) {
								duckLocation = 1;
							} else if (duckLocation > 300.0) {
								duckLocation = 2;
							} else {
								duckLocation = 0;
							}
							telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
							telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
									recognition.getLeft(), recognition.getTop());
						} else {
							telemetry.addData("Non-duck object: ", recognition.getLabel());
							telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
									recognition.getLeft(), recognition.getTop());
						}
						i++;
					}
					telemetry.update();
				}
			}
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
				case 0: {
					robot.runAuto(Robot.AutonomousPath.RED_FAR_CAROUSEL_LEVEL_1_PARK_1_TRAJECTORY, this);
				} break;

				case 1: {
					robot.runAuto(Robot.AutonomousPath.RED_FAR_CAROUSEL_LEVEL_2_PARK_1_TRAJECTORY, this);
				} break;

				case 2: {
					robot.runAuto(Robot.AutonomousPath.RED_FAR_CAROUSEL_LEVEL_3_PARK_1_TRAJECTORY, this);
				} break;
			}
			requestOpModeStop();
		}
	}

	/**
	 * Initialize the Vuforia localization engine.
	 */
	private void initVuforia() {
		/*
		 * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
		 */
		VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

		parameters.vuforiaLicenseKey = VUFORIA_KEY;
		parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

		//  Instantiate the Vuforia engine
		vuforia = ClassFactory.getInstance().createVuforia(parameters);

		// Loading trackables is not necessary for the TensorFlow Object Detection engine.
	}

	/**
	 * Initialize the TensorFlow Object Detection engine.
	 */
	private void initTfod() {
		int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
				"tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
		tfodParameters.minResultConfidence = 0.8f;
		tfodParameters.isModelTensorFlow2 = true;
		tfodParameters.inputSize = 320;
		tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
		tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
	}

}
