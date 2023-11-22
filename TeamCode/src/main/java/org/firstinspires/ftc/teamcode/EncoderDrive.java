package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;



    /*
     * This OpMode illustrates the concept of driving a path based on encoder counts.
     * The code is structured as a LinearOpMode
     *
     * The code REQUIRES that you DO have encoders on the wheels,
     *   otherwise you would use: RobotAutoDriveByTime;
     *
     *  This code ALSO requires that the drive Motors have been configured such that a positive
     *  power command moves them forward, and causes the encoders to count UP.
     *
     *   The desired path in this example is:
     *   - Drive forward for 48 inches
     *   - Spin right for 12 Inches
     *   - Drive Backward for 24 inches
     *   - Stop and close the claw.
     *
     *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
     *  that performs the actual movement.
     *  This method assumes that each movement is relative to the last stopping place.
     *  There are other ways to perform encoder based moves, but this method is probably the simplest.
     *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
     *
     * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
     * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
     */

    @Autonomous(name="EncoderDrive", group="LinearOpMode")
//    @Disabled
    public class EncoderDrive extends LinearOpMode {

        /* Declare OpMode members. */
//        private LinearOpMode myOpMode = this;   // gain access to methods in the calling OpMode.
        private DcMotor leftFrontDrive = null;
        private DcMotor leftBackDrive = null;
        private DcMotor rightFrontDrive = null;
        private DcMotor rightBackDrive = null;

        private ElapsedTime runtime = new ElapsedTime();

        // Calculate the COUNTS_PER_INCH for your specific drive train.
        // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
        // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
        // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
        // This is gearing DOWN for less speed and more torque.
        // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
        // https://www.gobilda.com/5202-series-yellow-jacket-planetary-gear-motor-13-7-1-ratio-24mm-length-6mm-d-shaft-435-rpm-36mm-gearbox-3-3-5v-encoder/
        // gobilda 5202, 435 rpm
        static final double     COUNTS_PER_MOTOR_REV    = 384.5 ;
        // Gear - https://www.gobilda.com/2-1-ratio-bevel-gear-set-6mm-d-bore-pinion-gear/
        static final double     DRIVE_GEAR_REDUCTION    = 2;
        static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
        static final double     COUNTS_PER_INCH         =(COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);
        static final double     DRIVE_SPEED             = 0.5;
        static final double     TURN_SPEED              = 0.5;

        @Override
        public void runOpMode() {

//            myOpMode = this;
            // Initialize the drive system variables.
            leftFrontDrive = hardwareMap.get(DcMotor.class, "fl"); // controller port 0
            leftBackDrive = hardwareMap.get(DcMotor.class, "bl"); // controller port 1
            rightFrontDrive = hardwareMap.get(DcMotor.class, "fr"); // controller port 2
            rightBackDrive = hardwareMap.get(DcMotor.class, "br"); // controller port 3
            
            // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
            // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
            // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
            leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Send telemetry message to indicate successful Encoder reset
            telemetry.addData("Starting at",  "%7d :%7d",
                    leftFrontDrive.getCurrentPosition(),
                    rightFrontDrive.getCurrentPosition(),
                    leftBackDrive.getCurrentPosition(),
                    rightBackDrive.getCurrentPosition());
            telemetry.update();

            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            double distanceToMove = 21.5; // inches
            // Step through each leg of the path,
            // Note: Reverse movement is obtained by setting a negative distance (not speed)

            for (int i = 0; i < 4; i++) {
                //Move backward
                //encoderDrive(DRIVE_SPEED, -distanceToMove, -distanceToMove, -distanceToMove, -distanceToMove, 5);
                //Turn Left
                encoderDrive(TURN_SPEED,-distanceToMove,distanceToMove,-distanceToMove,distanceToMove,5);
                sleep (500);
                //Move forward
                //encoderDrive(DRIVE_SPEED, distanceToMove, distanceToMove, distanceToMove, distanceToMove, 5);


                telemetry.addData("Path", "Complete");
                telemetry.update();
                sleep(500);  // pause to display final telemetry message.
            }
        }

        /*
         *  Method to perform a relative move, based on encoder counts.
         *  Encoders are not reset as the move is based on the current position.
         *  Move will stop if any of three conditions occur:
         *  1) Move gets to the desired position
         *  2) Move runs out of time
         *  3) Driver stops the OpMode running.
         */
        public void encoderDrive(double speed,
                                 double leftFrontInches, double rightFrontInches,double leftBackInches, double rightBackInches,
                                 double timeoutS) {
            int newLeftFrontTarget;
            int newRightFrontTarget;
            int newLeftBackTarget;
            int newRightBackTarget;

            // Ensure that the OpMode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH);
                newRightFrontTarget = rightFrontDrive.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
                newLeftBackTarget = leftBackDrive.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
                newRightBackTarget = rightBackDrive.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);

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
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() && leftBackDrive.isBusy() && rightBackDrive.isBusy())) {

                    // Display it for the driver.
                    telemetry.addData("Running to",  " %7d :%7d", newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
                    telemetry.addData("Currently at",  " at %7d :%7d",
                            leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
                    telemetry.update();
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
    }


