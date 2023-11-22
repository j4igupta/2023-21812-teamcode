package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import com.acmerobotics.dashboard.config.Config;

import java.util.List;


/**
 * This file illustrates the concept of driving up to a line and then stopping.
 * The code is structured as a LinearOpMode
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

/**
 * Jai Gupta 10/18/2023
 * This is an outline of the code - fill it in with actual code and it will work (hopefully)
 * based on blueBackdrop
 * Move forward 1 block
 * turn right 90 degrees
 * scan area : if green place pixel, place purple and turn left 180 degrees
 * move 2 blocks forward
 * <p>
 * else turn left 90 degrees
 **/
@Config
@Autonomous(name = "LeftLandingBlueClose", group = "Linear OpMode")
public class AutoOpCenterStageLandingPositionLeftBlueClose extends LinearOpMode {

    Robot robot = new Robot(this);
    //static final double     FORWARD_SPEED = 0.35;
    private final ElapsedTime runtime = new ElapsedTime();
    int StrafeTime = 1500;
//    boolean pixelDetected = false;
    public static double TURN90 = 21.75; //change based on test results
    public static double TURN180 = TURN90 * 2; //change based on test results
    public static boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    private TfodProcessor tfod;
    private static final double OBJECT_DETECT_CONFIDENCE = 0.7;
    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        robot.init();
        initTfod();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                goToScanPosition();
//                telemetry.addData("abc", "def");
                int playId = determinePlayId();
                //TODO: turn off streaming before final
//                executePlay(playId);
            }
        }
        visionPortal.close();

    }

    void goToScanPosition() {
        //Code to go to scan position
        robot.driveForward(-10);
        sleep(2000);
//        if (pixelDetected) {
//            telemetry.addData("Pixel", "Detected");
//            telemetry.update();
//            robot.driveBackward(-10);
//        }
        sleep(30000);
    }


    int determinePlayId() {

        int id = 1;
        // Code to determine which id to play
        // by scanning for team prop
        // left is 0, center is 1, right is 2
        //check every possibility
        boolean pixelDetected = detectPixel();
        if (pixelDetected) {
            //middle
            telemetry.addData("Pixel detected at", "middle");
            telemetry.update();
            return id = 1;

        } else {
            //left
            robot.turnLeft((float) -TURN90);
            pixelDetected = detectPixel();
            if (pixelDetected) {
                telemetry.addData("Pixel detected at", "left");
                telemetry.update();
                return id = 0;

            } else {
                robot.turnLeft((float) -TURN180);
// if pixel is not detected in the first two, assume it is in the last one, so no need to check
                telemetry.addData("Pixel detected at", "right");
                telemetry.update();
                return id = 2;
            }
        }
    }

    //TODO: fix distances according to test results
    //TODO: make duplicate file for landingpositionleft
    void executePlay(int playId) {
        // code to execute play
        int strafePosition = 1;
        switch (playId) {
            case 0:
                //code for left
                robot.driveForward(-3);
                sleep(50);
                robot.driveBackward(-50);
                sleep(50);
                robot.turnLeft((float) -TURN90);
                sleep(50);
                //TODO:return strafePosition = 18;

                break;
            case 1:
                //code for middle
                robot.driveForward(-3);
                sleep(50);
                robot.driveBackward(-3);
                sleep(50);
                robot.turnLeft((float) -TURN180);
                //TODO:return strafePosition = 24
                break;
            case 2:
                //code for right
                robot.driveForward(-3);
                sleep(50);
                robot.driveBackward(-3);
                sleep(50);
                robot.turnRight((float) -TURN90);
                sleep(50);
                //TODO:return strafePosition = 30;
                break;
        }
        robot.driveForward(-24);
        sleep(50);
        robot.turnRight((float) -TURN90);
        sleep(50);
        robot.driveForward(-36);
        sleep(50);
        robot.strafeRight(-strafePosition);
        sleep(50);
        //TODO:put output pixel code here
        robot.strafeLeft(-strafePosition);
        sleep(50);
        robot.driveForward(-24);
    }

    private void initTfod() {

        // Create the TensorFlow processor the easy way.
        tfod = TfodProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, tfod);
        }

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private boolean detectPixel() {
        sleep(1000);

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            telemetry.update();
            if (recognition.getConfidence() > OBJECT_DETECT_CONFIDENCE) {
                return true;
            }

        }   // end for() loop

        return false;
    }   // end method telemetryTfod()

}


