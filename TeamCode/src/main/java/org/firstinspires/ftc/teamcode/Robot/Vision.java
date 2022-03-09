package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class Vision {
	private double position = 0.0f;
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

	private HardwareMap HWMAP = null;


	public Vision(HardwareMap hwMap) {
		HWMAP = hwMap;
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

	}

	public double update() {
		if (tfod != null) {
			// getUpdatedRecognitions() will return null if no new information is available since
			// the last time that call was made.
			List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
			if (updatedRecognitions != null) {
				position = -1.0;
				// step through the list of recognitions and display boundary info.
				for (Recognition recognition : updatedRecognitions) {
					if (recognition.getLabel().equals("Duck") || recognition.getLabel().equals("Cube")) {
						position = recognition.getLeft();
						return position;
					}
				}
			} else {
				return position;
			}
		}
		return position;
	}

	private void initVuforia() {
		/*
		 * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
		 */
		VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

		parameters.vuforiaLicenseKey = VUFORIA_KEY;
		parameters.cameraName = HWMAP.get(WebcamName.class, "Webcam 1");

		//  Instantiate the Vuforia engine
		vuforia = ClassFactory.getInstance().createVuforia(parameters);

		// Loading trackables is not necessary for the TensorFlow Object Detection engine.
	}

	/**
	 * Initialize the TensorFlow Object Detection engine.
	 */
	private void initTfod() {
		int tfodMonitorViewId = HWMAP.appContext.getResources().getIdentifier(
				"tfodMonitorViewId", "id", HWMAP.appContext.getPackageName());
		TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
		tfodParameters.minResultConfidence = 0.8f;
		tfodParameters.isModelTensorFlow2 = true;
		tfodParameters.inputSize = 320;
		tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
		tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
	}
}
