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

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "3 foot auto 90 degree turn #2", group = "Robosapiens")
public class EncodersAndLogging extends LinearOpMode {

    private static final String TAG = "ENCODERS";

    MyHardwarePushbot robot = new MyHardwarePushbot();
    private ElapsedTime runtime = new ElapsedTime();

    static final double FORWARD_ENCODER_COUNTS_PER_INCH = 116;
    static final double LATERAL_ENCODER_COUNTS_PER_INCH = 141.15;
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            initialize();
            waitForStart();
            //push the ball
            pushball();
            //go forward 2 feet
            moveForwardBy(36.0);
            //turn 90 %
            turnBy(19);

            sleep(2000);

            //shift 12 inches
            moveLateralBy(12.0);


            log("Program Complete");
        } finally {
            robot.resetMotors();
        }
    }

    private void moveForwardBy(double inches) {
        encoderForward(DRIVE_SPEED, inches);
        moveForUpTo(5.0);
        robot.resetMotors();
    }

    private void moveLateralBy(double inches) {
        encoderLateral(DRIVE_SPEED, inches);
        moveForUpTo(5.0);
        robot.resetMotors();
    }


    private void initialize() {
        robot.init(hardwareMap);
        log("initialization complete");
        logEncoderPosition();
    }

    private void turnBy(double inches) {
        encoderTurn(TURN_SPEED, inches);
        moveForUpTo(3.0);
        robot.resetMotors();
    }
    private void pushball(){
        //puts down servo (color holder)
        robot.colorHolder.setPosition(MyHardwarePushbot.MID_SERVO);
        //stops for 2 seconds
        sleep(2000);

        NormalizedRGBA colors = robot.colorSensor.getNormalizedColors();
       // if red ball, do this
        if(isRed(colors)){
            //goes forward
            turnBy(9.0);

            //stops 2 sec
            sleep(2000);

            //puts up color holder
            robot.colorHolder.setPosition(MyHardwarePushbot.MIN_SERVO);

            //stops 2 sec
            sleep(2000);

            turnBy(-9.0);

        } else if(isBlue(colors)){//if blue ball
            //goes back
            moveForwardBy(5.0);

            //stops for 2 sec
            sleep(2000);

            //puts up color holder
            robot.colorHolder.setPosition(MyHardwarePushbot.MIN_SERVO);

            //stops 2 sec
            sleep(2000);

            //goes forward 6
            moveForwardBy(-5.0);

            //stop 2 sec
            sleep(2000);
        }else{
            //did not find red or blue raise the color sensor holder and move on
            robot.colorHolder.setPosition(MyHardwarePushbot.MIN_SERVO);
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
    private void encoderForward(double speed,
                                double inches) {
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            robot.rightDrive.setDirection(DcMotor.Direction.FORWARD);
            robot.leftDrive.setDirection(DcMotor.Direction.REVERSE);
            robot.rightbackDrive.setDirection(DcMotor.Direction.FORWARD);
            robot.leftbackDrive.setDirection(DcMotor.Direction.REVERSE);
            setToMoveBy(inches, FORWARD_ENCODER_COUNTS_PER_INCH);
            setToMoveAt(speed);
        }
    }

    public void encoderLateral(double speed,
                               double inches) {
        if (opModeIsActive()) {
            robot.leftbackDrive.setDirection(DcMotor.Direction.REVERSE);
            robot.rightbackDrive.setDirection(DcMotor.Direction.REVERSE);
            robot.leftDrive.setDirection(DcMotor.Direction.FORWARD);
            robot.rightDrive.setDirection(DcMotor.Direction.FORWARD);
            setToMoveBy(inches, LATERAL_ENCODER_COUNTS_PER_INCH);
            setToMoveAt(speed);
        }
    }
    public void encoderTurn(double speed,
                            double inches){
        if (opModeIsActive()){
            robot.leftbackDrive.setDirection(DcMotor.Direction.FORWARD);
            robot.rightbackDrive.setDirection(DcMotor.Direction.REVERSE);
            robot.leftDrive.setDirection(DcMotor.Direction.FORWARD);
            robot.rightDrive.setDirection(DcMotor.Direction.REVERSE);
            setToTurnBy(inches, LATERAL_ENCODER_COUNTS_PER_INCH);
            setToMoveAt(speed);

        }
    }

    public boolean isRed(NormalizedRGBA colors){

        return colors.red > colors.blue  && colors.red > colors.green && colors.blue >= 0.001 && colors.green >= 0.001;
    }

    public boolean isBlue(NormalizedRGBA colors){

        return colors.blue > colors.red && colors.blue >= colors.green;
    }

    private void setToMoveBy(double inches, double encoderCountsPerInch) {
        log("Starting Encoder Position");
        logEncoderPosition();
        int newLeftTarget = robot.leftDrive.getCurrentPosition() + (int) (inches * encoderCountsPerInch);
        int newRightTarget = robot.rightDrive.getCurrentPosition() + (int) (inches * encoderCountsPerInch);
        int newLeftbackTarget = robot.leftbackDrive.getCurrentPosition() + (int) (inches * encoderCountsPerInch);
        int newRightbackTarget = robot.rightbackDrive.getCurrentPosition() + (int) (inches * encoderCountsPerInch);

        // Turn On RUN_TO_POSITION
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftbackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightbackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftDrive.setTargetPosition(newLeftTarget);
        robot.rightDrive.setTargetPosition(newRightTarget);
        robot.leftbackDrive.setTargetPosition(newLeftbackTarget);
        robot.rightbackDrive.setTargetPosition(newRightbackTarget);
        logEncoderTargetPosition();
    }


    private void setToTurnBy(double inches, double encoderCountsPerInch) {
        log("Starting Encoder Position");
        logEncoderPosition();
        int newLeftTarget = robot.leftDrive.getCurrentPosition() + (int) (-inches * encoderCountsPerInch);
        int newRightTarget = robot.rightDrive.getCurrentPosition() + (int) (inches * encoderCountsPerInch);
        int newLeftbackTarget = robot.leftbackDrive.getCurrentPosition() + (int) (-inches * encoderCountsPerInch);
        int newRightbackTarget = robot.rightbackDrive.getCurrentPosition() + (int) (inches * encoderCountsPerInch);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftbackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightbackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftDrive.setTargetPosition(newLeftTarget);
        robot.rightDrive.setTargetPosition(newRightTarget);
        robot.leftbackDrive.setTargetPosition(newLeftbackTarget);
        robot.rightbackDrive.setTargetPosition(newRightbackTarget);
        logEncoderTargetPosition();
    }

    private void setToMoveAt(double speed) {
        robot.leftDrive.setPower(Math.abs(speed));
        robot.rightDrive.setPower(Math.abs(speed)+.01);
        robot.leftbackDrive.setPower(Math.abs(speed));
        robot.rightbackDrive.setPower(Math.abs(speed)+.01);
    }

    private void moveForUpTo(double timeoutS) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                (robot.leftDrive.isBusy() && robot.rightDrive.isBusy() && robot.leftbackDrive.isBusy() && robot.rightbackDrive.isBusy())){
            log("Changing Encoder Position");
            logEncoderPosition();
        }
        log("Final Encoder Position");
        logEncoderPosition();
    }

    private void log(String message, Throwable... throwables) {
        if (throwables.length > 0){
            Log.d(TAG, message, throwables[0]);
        } else {
            Log.d(TAG, message);
        }
        telemetry.addLine(TAG + ": " + message);
        telemetry.update();
    }

    private void logEncoderPosition() {
        log(String.format("Encoders at %7d :%7d :%7d :%7d",
                robot.leftDrive.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition(),
                robot.leftbackDrive.getCurrentPosition(),
                robot.rightbackDrive.getCurrentPosition()));
    }

    private void logEncoderTargetPosition() {
        log(String.format("Target Postion at %7d :%7d :%7d :%7d",
                robot.leftDrive.getTargetPosition(),
                robot.rightDrive.getTargetPosition(),
                robot.leftbackDrive.getTargetPosition(),
                robot.rightbackDrive.getTargetPosition()));
    }
}
