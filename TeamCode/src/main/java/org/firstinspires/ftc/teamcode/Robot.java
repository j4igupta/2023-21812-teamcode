package org.firstinspires.ftc.teamcode;


import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Robot {
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private Servo droneLauncher = null;
    private Servo left_hand = null;
    private Servo right_hand = null;
    private DcMotor linearSlideRight = null;
    private DcMotor linearSlideLeft = null;
    private DcMotor gripperWheels = null;
    static final double FORWARD_SPEED = 0.35;
    static final double linearSlideRightPower = 0.4;
    static final double linearSlideLeftPower = 0.4;
    static final double gripperWheelsPower = 1;
    static final double COUNTS_PER_MOTOR_REV = 384.5;
    // Gear - https://www.gobilda.com/2-1-ratio-bevel-gear-set-6mm-d-bore-pinion-gear/
    static final double DRIVE_GEAR_REDUCTION = 2;
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.5;
    private ElapsedTime runtime = new ElapsedTime();

    public Robot(LinearOpMode OpMode) {
        myOpMode = OpMode;
    }


    public void init() {
        // Initialize the drive system variables.
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "fl"); // controller port 0
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "bl"); // controller port 1
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "fr"); // controller port 2
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "br"); // controller port 3
        linearSlideRight = myOpMode.hardwareMap.get(DcMotor.class, "slideRight"); // expansion port 0
        linearSlideLeft = myOpMode.hardwareMap.get(DcMotor.class, "slideLeft"); // expansion port 1
        gripperWheels = myOpMode.hardwareMap.get(DcMotor.class, "gripper");//expansion port 2
        // Define and initialize ALL installed servos.
        droneLauncher = myOpMode.hardwareMap.get(Servo.class, "droneLauncher");//expansion servo 0
        left_hand = myOpMode.hardwareMap.get(Servo.class, "left_hand");//expansion servo 1
        right_hand = myOpMode.hardwareMap.get(Servo.class, "right_hand");//expansion servo 2

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);


        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        linearSlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gripperWheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        droneLauncher.setPosition(1);
        left_hand.setPosition(1);
        right_hand.setPosition(0);
        // Wait for the game to start (driver presses PLAY)
        myOpMode.telemetry.addData("Status", "Initialized");
        myOpMode.telemetry.addData("Starting at", "%7d :%7d",
                leftFrontDrive.getCurrentPosition(),
                rightFrontDrive.getCurrentPosition(),
                leftBackDrive.getCurrentPosition(),
                rightBackDrive.getCurrentPosition());
        myOpMode.telemetry.update();

    }

    public void encoderDrive(double speed, double leftFrontInches, double rightFrontInches,
                             double leftBackInches,
                             double rightBackInches, double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the OpMode is still active
        if (myOpMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int) (leftFrontInches * COUNTS_PER_INCH);
            newRightFrontTarget = rightFrontDrive.getCurrentPosition() + (int) (rightFrontInches * COUNTS_PER_INCH);
            newLeftBackTarget = leftBackDrive.getCurrentPosition() + (int) (leftBackInches * COUNTS_PER_INCH);
            newRightBackTarget = rightBackDrive.getCurrentPosition() + (int) (rightBackInches * COUNTS_PER_INCH);

            leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            rightFrontDrive.setTargetPosition(newRightFrontTarget);
            leftBackDrive.setTargetPosition(newLeftBackTarget);
            rightBackDrive.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFrontDrive.setPower(Math.abs(speed));
            rightFrontDrive.setPower(Math.abs(speed));
            leftBackDrive.setPower(Math.abs(speed));
            rightBackDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (myOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() && leftBackDrive.isBusy() && rightBackDrive.isBusy())) {

                // Display it for the driver.
                myOpMode.telemetry.addData("Running to", " %7d :%7d", newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
                myOpMode.telemetry.addData("Currently at", " at %7d :%7d",
                        leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
                myOpMode.telemetry.update();
            }

            // Stop all motion;
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
            // Turn off RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }

    public void setDrivePower(double leftFront, double rightFront, double leftBack, double rightBack) {
        // Output the values to the motor drives.
        leftFrontDrive.setPower(leftFront);
        rightFrontDrive.setPower(rightFront);
        leftBackDrive.setPower(leftBack);
        rightBackDrive.setPower(rightBack);
    }

    public void launchDrone() {
        droneLauncher.setPosition(0.5);
    }

    public void unlaunchDrone() {
        droneLauncher.setPosition(1);
    }

    public void openClaw(){
        left_hand.setPosition(0);
        right_hand.setPosition(0.07);
    }
    public void closeClaw(){
        left_hand.setPosition(0.07);
        right_hand.setPosition(0);
    }

    public void linearSlidePower(double power) {
        linearSlideRight.setPower(power);
        linearSlideLeft.setPower(-power);
    }

    public void gripperWheelsPower(double gripperWheelsPower) {
        gripperWheels.setPower(gripperWheelsPower);
    }
//TODO drive all drive should be double
    public void driveForward(int inches) {
        encoderDrive(DRIVE_SPEED,inches,inches,inches,inches,5);
    }

    public void driveBackward(int inches) {
        encoderDrive(DRIVE_SPEED,-inches, -inches, -inches, -inches,5);
    }

    public void turnRight(float inches) {
        encoderDrive(DRIVE_SPEED,-inches,inches,-inches,inches,5);
    }

    public void turnLeft (float inches) {
        encoderDrive(DRIVE_SPEED,inches,-inches,inches,-inches,5);
    }
    public void stopRobot() {
        setDrivePower(0, 0, 0, 0);
        stopSlides();
        stopWheels();
    }


    public void stopWheels() {
        setDrivePower(0, 0, 0, 0);
    }

    public void stopSlides() {
        linearSlideRight.setPower(0);
        linearSlideLeft.setPower(0);
    }

    public void stopGripperWheels() {
        gripperWheels.setPower(0);
    }

    public void strafeLeft(int inches) {
        encoderDrive(DRIVE_SPEED,-inches,inches,inches,-inches,5);
    }

    public void strafeRight(int inches) {
        encoderDrive(DRIVE_SPEED,inches, -inches, -inches, inches,5);
    }
}






