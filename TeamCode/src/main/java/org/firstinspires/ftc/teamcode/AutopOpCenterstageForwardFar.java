package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;


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
@Autonomous(name = "ForwardFar", group = "Linear OpMode")
public class AutopOpCenterstageForwardFar extends LinearOpMode {

    Robot robot = new Robot(this);
    //static final double     FORWARD_SPEED = 0.35;
    private final ElapsedTime runtime = new ElapsedTime();
    int StrafeTime = 1500;
    boolean pixelDetected = true;
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
        robot.strafeRight(-3);
        robot.driveForward(-125);
        sleep(100);
    }

}


