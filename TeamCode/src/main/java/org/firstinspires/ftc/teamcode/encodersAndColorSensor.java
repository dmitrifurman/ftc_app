/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.SensorColor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.teamcode.MyHardwarePushbot;

import static com.qualcomm.robotcore.R.attr.colors;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="wifi auto test with color sensor 7.", group="Pushbot")
@Disabled
public class encodersAndColorSensor extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot         robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = .012;    // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = 116;

    //static final double     COUNTS_PER_INCH          = 0.267;

    static final double     DRIVE_SPEED             = 0.6;
    static final double  TURN_SPEED = 0.5;


    /** The relativeLayout field is used to aid in providing interesting visual feedback
     * in this sample application; you probably *don't* need something analogous when you
     * use a color sensor on your robot */
    View relativeLayout;
    @Override
    public void runOpMode() throws InterruptedException {


        try {
            initialize();//initialize motors
            runWithSensor(); // start and read color sensor and use the value to start motors
        } finally {
            // On the way out, *guarantee* that the background is reasonable. It doesn't actually start off
            // as pure white, but it's too much work to dig out what actually was used, and this is good
            // enough to at least make the screen reasonable again.
            // Set the panel back to the default color
//            relativeLayout.post(new Runnable() {
//                public void run() {
//                    relativeLayout.setBackgroundColor(Color.WHITE);
//                }
//            });
        }


    /*    robot.leftClaw.setPosition(1.0);            // S4: Stop and close the claw.
        robot.rightClaw.setPosition(0.0);
        sleep(1000);     // pause for servos to move
*/
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    private void initialize () {

               /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        //colorHolder = hardwareMap.get(Servo.class, "colorHolder");
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.addData("COUNTS_PER_INCH = " , COUNTS_PER_INCH);
        telemetry.update();


        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftbackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightbackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftbackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightbackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.leftDrive.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition(),
                robot.leftbackDrive.getCurrentPosition(),
                robot.rightbackDrive.getCurrentPosition());
        telemetry.update();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }

    protected void runWithSensor() throws InterruptedException {


        // values is a reference to the hsvValues array.
        float[] hsvValues = new float[3];
        final float values[] = hsvValues;

        NormalizedRGBA colors = robot.colorSensor.getNormalizedColors();
        // Wait for the start button to be pressed.
        waitForStart();

        // Loop until we are asked to stop
        while (opModeIsActive()) {


            /** Use telemetry to display feedback on the driver station. We show the conversion
             * of the colors to hue, saturation and value, and display the the normalized values
             * as returned from the sensor.
             * @see <a href="http://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html">HSV</a>*/

            Color.colorToHSV(colors.toColor(), hsvValues);
            telemetry.addLine()
                    .addData("H", "%.3f", hsvValues[0])
                    .addData("S", "%.3f", hsvValues[1])
                    .addData("V", "%.3f", hsvValues[2]);
            telemetry.addLine()
                    .addData("a", "%.3f", colors.alpha)
                    .addData("r", "%.3f", colors.red)
                    .addData("g", "%.3f", colors.green)
                    .addData("b", "%.3f", colors.blue);

            boolean isRed = colors.red >= colors.blue*1.33 && colors.red >= colors.green*1.33;
            boolean isBlue = colors.blue > colors.red && colors.blue >= colors.green;

            telemetry.addData("isred", isRed);
            telemetry.addData("isblue", isBlue);
            telemetry.update();

            // Step through each leg of the path,
            // Note: Reverse movement is obtained by setting a negative distance (not speed)
            robot.colorHolder.setPosition(robot.MIN);
            //if red run red routine
            if (isRed) {
                sleep(1000);
                robot.leftDrive.setDirection(DcMotor.Direction.REVERSE);
                robot.rightDrive.setDirection(DcMotor.Direction.FORWARD);
                robot.leftbackDrive.setDirection(DcMotor.Direction.REVERSE);
                robot.rightbackDrive.setDirection(DcMotor.Direction.FORWARD);

                sleep(1000);

                encoderDrive(DRIVE_SPEED, 6, 6, 5.0);  // S1: Forward 1 feet with 5 Sec timeout

                sleep(1000);

                robot.colorHolder.setPosition(robot.MAX);//raise arm

                sleep(1000);

                encoderDrive(TURN_SPEED, 18, 18, 5.0);

                sleep(1000);

                robot.leftDrive.setDirection(DcMotor.Direction.FORWARD);
                robot.rightDrive.setDirection(DcMotor.Direction.FORWARD);
                robot.leftbackDrive.setDirection(DcMotor.Direction.REVERSE);
                robot.rightbackDrive.setDirection(DcMotor.Direction.REVERSE);

                sleep(1000);

                encoderDrive(DRIVE_SPEED,  24, 24, 5.0);

                sleep(1000);

                robot.leftDrive.setDirection(DcMotor.Direction.REVERSE);
                robot.rightDrive.setDirection(DcMotor.Direction.FORWARD);
                robot.leftbackDrive.setDirection(DcMotor.Direction.REVERSE);
                robot.rightbackDrive.setDirection(DcMotor.Direction.FORWARD);

                sleep(1000);

                encoderDrive(TURN_SPEED, 25.5, -25.5, 5.0);

                sleep(24000);
               /* encoderDrive(TURN_SPEED, 10.32, -10.32, 5.0);  // S2: Turn Right 3 Inches with 4 Sec timeout

                encoderDrive(DRIVE_SPEED, 12, 12, 5.0);  // S3: forward 1 foot with 4 Sec timeout
                encoderDrive(TURN_SPEED, -20.64, 20.64, 5.0);
                robot.leftDrive.setDirection(DcMotor.Direction.FORWARD);
                robot.rightDrive.setDirection(DcMotor.Direction.FORWARD);
                robot.leftbackDrive.setDirection(DcMotor.Direction.REVERSE);
                robot.rightbackDrive.setDirection(DcMotor.Direction.REVERSE);
                encoderDrive(DRIVE_SPEED, 12, 12, 5.0);
                robot.leftDrive.setDirection(DcMotor.Direction.REVERSE);
                robot.rightDrive.setDirection(DcMotor.Direction.FORWARD);
                robot.leftbackDrive.setDirection(DcMotor.Direction.REVERSE);
                robot.rightbackDrive.setDirection(DcMotor.Direction.FORWARD);
                encoderDrive(DRIVE_SPEED, 12, 12, 5.0);*/

            } else if(isBlue){
                sleep(1000);
                robot.leftDrive.setDirection(DcMotor.Direction.FORWARD);
                robot.rightDrive.setDirection(DcMotor.Direction.REVERSE);
                robot.leftbackDrive.setDirection(DcMotor.Direction.FORWARD);
                robot.rightbackDrive.setDirection(DcMotor.Direction.REVERSE);

                sleep(1000);

                encoderDrive(DRIVE_SPEED, 6, 6, 5.0);  // S1: Forward 1 feet with 5 Sec timeout

                sleep(1000);

                robot.colorHolder.setPosition(robot.MAX);//raise arm

                sleep(1000);

                robot.leftDrive.setDirection(DcMotor.Direction.REVERSE);
                robot.rightDrive.setDirection(DcMotor.Direction.FORWARD);
                robot.leftbackDrive.setDirection(DcMotor.Direction.REVERSE);
                robot.rightbackDrive.setDirection(DcMotor.Direction.FORWARD);

                encoderDrive(TURN_SPEED, 30, 30, 5.0);

                sleep(1000);

                robot.leftDrive.setDirection(DcMotor.Direction.REVERSE);
                robot.rightDrive.setDirection(DcMotor.Direction.REVERSE);
                robot.leftbackDrive.setDirection(DcMotor.Direction.FORWARD);
                robot.rightbackDrive.setDirection(DcMotor.Direction.FORWARD);

                sleep(1000);

                encoderDrive(DRIVE_SPEED,  24, 24, 5.0);

                sleep(1000);

                robot.leftDrive.setDirection(DcMotor.Direction.REVERSE);
                robot.rightDrive.setDirection(DcMotor.Direction.FORWARD);
                robot.leftbackDrive.setDirection(DcMotor.Direction.REVERSE);
                robot.rightbackDrive.setDirection(DcMotor.Direction.FORWARD);

                sleep(1000);

                encoderDrive(TURN_SPEED, -25.5, 25.5, 5.0);

                sleep(24000);

            }
            robot.leftDrive.setDirection(DcMotor.Direction.REVERSE);
            robot.rightDrive.setDirection(DcMotor.Direction.FORWARD);
            robot.leftbackDrive.setDirection(DcMotor.Direction.REVERSE);
            robot.rightbackDrive.setDirection(DcMotor.Direction.FORWARD);

            /** We also display a conversion of the colors to an equivalent Android color integer.
             * @see Color */
            int color = colors.toColor();
            telemetry.addLine("raw Android color: ")
                    .addData("a", "%02x", Color.alpha(color))
                    .addData("r", "%02x", Color.red(color))
                    .addData("g", "%02x", Color.green(color))
                    .addData("b", "%02x", Color.blue(color));

            // Balance the colors. The values returned by getColors() are normalized relative to the
            // maximum possible values that the sensor can measure. For example, a sensor might in a
            // particular configuration be able to internally measure color intensity in a range of
            // [0, 10240]. In such a case, the values returned by getColors() will be divided by 10240
            // so as to return a value it the range [0,1]. However, and this is the point, even so, the
            // values we see here may not get close to 1.0 in, e.g., low light conditions where the
            // sensor measurements don't approach their maximum limit. In such situations, the *relative*
            // intensities of the colors are likely what is most interesting. Here, for example, we boost
            // the signal on the colors while maintaining their relative balance so as to give more
            // vibrant visual feedback on the robot controller visual display.
            float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
            colors.red   /= max;
            colors.green /= max;
            colors.blue  /= max;
            color = colors.toColor();

            telemetry.addLine("normalized color:  ")
                    .addData("a", "%02x", Color.alpha(color))
                    .addData("r", "%02x", Color.red(color))
                    .addData("g", "%02x", Color.green(color))
                    .addData("b", "%02x", Color.blue(color));



            // convert the RGB values to HSV values.
            Color.RGBToHSV(Color.red(color), Color.green(color), Color.blue(color), hsvValues);

            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });



        }
    }
    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newLeftbackTarget;
        int newRightbackTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH );
            newRightTarget = robot.rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH );
            newLeftbackTarget = robot.leftbackDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH );
            newRightbackTarget = robot.rightbackDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH );

            telemetry.addData("COUNTS_PER_INCH2",COUNTS_PER_INCH);
            telemetry.addData("robot.leftDrive.getCurrentPosition()", robot.leftDrive.getCurrentPosition());
            telemetry.addData("robot.leftbackDrive.getCurrentPosition()", robot.leftbackDrive.getCurrentPosition());
            telemetry.addData("newLeftbackTarget", newLeftbackTarget);
            telemetry.addData("newLeftTarget", newLeftTarget);
            telemetry.update();


            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);
            robot.leftbackDrive.setTargetPosition(newLeftbackTarget);
            robot.rightbackDrive.setTargetPosition(newRightbackTarget);



            // Turn On RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftbackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightbackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDrive.setPower(Math.abs(speed));
            robot.rightDrive.setPower(Math.abs(speed)+0.01);
            robot.leftbackDrive.setPower(Math.abs(speed));
            robot.rightbackDrive.setPower(Math.abs(speed)+0.01);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.leftDrive.isBusy() && robot.rightDrive.isBusy() && robot.leftbackDrive.isBusy() && robot.rightbackDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d", newLeftTarget,  newRightTarget, newLeftbackTarget,newRightbackTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",
                                            robot.leftDrive.getCurrentPosition(),
                                            robot.rightDrive.getCurrentPosition(),
                        robot.leftbackDrive.getCurrentPosition(),
                        robot.rightbackDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);
            robot.leftbackDrive.setPower(0);
            robot.rightbackDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftbackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightbackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
