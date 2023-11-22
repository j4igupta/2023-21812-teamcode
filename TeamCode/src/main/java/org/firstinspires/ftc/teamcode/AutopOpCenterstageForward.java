package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import com.acmerobotics.dashboard.config.Config;

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
@Disabled
@Config
@Autonomous(name = "ForwardClose", group = "Linear OpMode")
public class AutopOpCenterstageForward extends LinearOpMode {

    Robot robot = new Robot(this);
    //static final double     FORWARD_SPEED = 0.35;
    private final ElapsedTime runtime = new ElapsedTime();
    int StrafeTime = 1500;
    boolean pixelDetected = true;
public static int FORWARD_MOVE_POSITION = 24;
  float turn90 = 20; //change based on test results
    float turn180 = turn90 * 2; //change based on test results
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    private TfodProcessor tfod;
    private static final double OBJECT_DETECT_CONFIDENCE = 0.7;
    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        robot.init();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                goToScanPosition();
                sleep(30000);

            }
        }


    }

    void goToScanPosition() {
        //Code to go to scan position
        robot.driveForward(FORWARD_MOVE_POSITION);
        sleep(100);
    }

}


