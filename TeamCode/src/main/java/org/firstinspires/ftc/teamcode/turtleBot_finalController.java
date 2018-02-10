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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.r
 *
 * Use Android Studios to Copy this Class, a
 * nd Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="turtle-bot v2.0 with relic grabber controller mode", group="Linear Opmode")
//@Disabled
public class turtleBot_finalController extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor leftbackDrive = null;
    private DcMotor rightbackDrive = null;
    private DcMotor elevator = null;
    private DcMotor spinner = null;
    private DcMotor extender = null;
    private Servo grableft;
    private Servo grabright;
    private Servo colorHolder;
    private Servo relicGrab;
    private static double SPEED = 1.0;

    // Maximum rotational position

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        rightbackDrive = hardwareMap.get(DcMotor.class, "rightback_drive");
        leftbackDrive = hardwareMap.get(DcMotor.class, "leftback_drive");
        spinner = hardwareMap.get(DcMotor.class, "spin");
        extender = hardwareMap.get(DcMotor.class, "extender");
        elevator = hardwareMap.get(DcMotor.class, "elevator");
        grableft = hardwareMap.get(Servo.class, "grableft");
        grabright = hardwareMap.get(Servo.class, "grabright");
        colorHolder = hardwareMap.get(Servo.class, "colorHolder");
        relicGrab = hardwareMap.get(Servo.class, "relicGrab");
        grableft.resetDeviceConfigurationForOpMode();
        grabright.resetDeviceConfigurationForOpMode();

        //Log.d("Turtle", "Init Left Power Status: "+ grableft.getController().getPwmStatus());
        //Log.d("Turtle", "Init Right Power Status: "+ grabright.getController().getPwmStatus());

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftbackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightbackDrive.setDirection(DcMotor.Direction.REVERSE);
        elevator.setDirection(DcMotor.Direction.FORWARD);
        spinner.setDirection(DcMotor.Direction.FORWARD);
        extender.setDirection(DcMotor.Direction.REVERSE);

        grabright.setDirection(Servo.Direction.REVERSE);
        grableft.setDirection(Servo.Direction.FORWARD);
        colorHolder.setDirection(Servo.Direction.REVERSE);
        relicGrab.setDirection(Servo.Direction.FORWARD);
        Log.d("Turtle", "Init Left: "+ grableft.getPosition());
        Log.d("Turtle", "Init Right: "+ grabright.getPosition());
        Log.d("Turtle", "Init Left Controller: "+ grableft.getController().getServoPosition(0));
        Log.d("Turtle", "Init Right Controller: "+ grabright.getController().getServoPosition(1));
        // Wait for the game to start (driver presses PLAY)
        boolean extended = false;
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double driveRight = gamepad1.right_stick_y;
            double driveLeft = gamepad1.left_stick_y;
            boolean spinright = gamepad2.b;
            boolean lift = gamepad2.y;
            boolean drop = gamepad2.a;
            double shiftLeft = gamepad1.left_stick_x;
            double shiftRight = gamepad1.right_stick_x;
            double leftgrab = gamepad2.left_stick_x;
            double rightgrab = gamepad2.right_stick_x;
            boolean spinleft = gamepad2.x;
            boolean moveForward = gamepad1.dpad_up;
            boolean moveBackward = gamepad1.dpad_down;
            boolean moveRight = gamepad1.dpad_right;
            boolean moveLeft = gamepad1.dpad_left;
            boolean extend = gamepad1.left_bumper;
            boolean retract = gamepad1.right_bumper;
            boolean relicHold = gamepad1.x;
            boolean stay = gamepad2.left_bumper;

            boolean caught = gamepad1.a;
            colorHolder.setPosition(0.4);

            if(extend) {
                extender.setPower(-0.75);
                extended = true;
            } else if(retract) {
                extender.setPower(0.75);
            } else {
                extender.setPower(0);
            }

            if(relicHold && !caught){
                relicGrab.setPosition(1.0);
            }else if(!relicHold && caught){
                relicGrab.setPosition(0.5);
            }

            if (moveForward){
                move(SPEED);
            } else {
                move(0.0);
            }
            if (moveBackward){
                move(-SPEED);
            } else {
                move(0.0);
            }
            if (moveRight){
                turn(SPEED);
            } else {
                turn(0.0);
            }
            if (moveLeft){
                turn(-SPEED);
            } else {
                turn(0.0);
            }

            if (driveRight > 0 || driveRight < 0){
                driveRight(SPEED * driveRight);
            } else {
                //leftDrive.setPower(0);
                driveRight(0);
            }
            if (driveLeft > 0 || driveLeft < 0){
                driveLeft(SPEED * driveLeft);
            } else {
                driveLeft(0);
            }

            double powerAdjust = 0.0;
            double powerUp = 0.0;
            if(extended) {
                powerAdjust = 0.05;
                powerUp = 0.25;
            }

            if (lift && !drop && !stay) {
                elevator.setPower(0.75 - powerUp);
            } else if (!lift && drop && !stay) {
                elevator.setPower(0.05 - powerAdjust);
                //} else if ((!lift && !drop) || (lift && drop)){
            } else if (!lift && !drop && stay){
                elevator.setPower(0.15);
            }

            while(leftgrab > 0 && rightgrab < 0) {
                closeLeftGrabSide();
                closeRightGrabSide();
                leftgrab = gamepad2.left_stick_x;
                rightgrab = gamepad2.right_stick_x;
            }

            while(leftgrab < 0 && rightgrab > 0) {
                openLeftGrabSide();
                openRightGrabSide();
                leftgrab = gamepad2.left_stick_x;
                rightgrab = gamepad2.right_stick_x;
            }

            // Open left side
            while (leftgrab < 0 && rightgrab == 0) {
                openLeftGrabSide();
                leftgrab = gamepad2.left_stick_x;
                rightgrab = gamepad2.right_stick_x;
            }

            // Close left side
            while (leftgrab > 0 && rightgrab == 0){
                //grabright.setPosition(MAX_GRAB);
                closeLeftGrabSide();
                leftgrab = gamepad2.left_stick_x;
                rightgrab = gamepad2.right_stick_x;
            }

            // Closed right side
            while (rightgrab < 0 && leftgrab == 0) {
                closeRightGrabSide();
                leftgrab = gamepad2.left_stick_x;
                rightgrab = gamepad2.right_stick_x;
            }

            // Open right side
            while (rightgrab > 0.5 && leftgrab == 0){
                openRightGrabSide();
                leftgrab = gamepad2.left_stick_x;
                rightgrab = gamepad2.right_stick_x;
            }

            if (spinright && !spinleft){
                spinner.setPower(0.8);
            } else if (spinleft && !spinright) {
                spinner.setPower(-0.8);
                //} else if ((!spinleft && !spinright) || (spinleft && spinright)){
            } else {
                spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                spinner.setPower(0);
            }

            if (shiftRight > 0){
                shiftRight(1.0);
            } else if (shiftRight < 0) {
                shiftRight(-1.0);
            } else {
                shiftRight(0);
            }
            if (shiftLeft > 0){
                shiftLeft(1.0);
                //rightbackDrive.setPower(0.5);
            } else if (shiftLeft < 0) {
                shiftLeft(-1.0);
            } else {
                shiftLeft(0);
            }
        }
    }

    private void shiftRight(double power) {
        //leftDrive.setPower(0.5);
        rightDrive.setPower(-power);
        //leftbackDrive.setPower(0.5);
        rightbackDrive.setPower(power);
    }

    private void shiftLeft(double power) {
        leftDrive.setPower(power);
        //rightDrive.setPower(0.5);
        leftbackDrive.setPower(-power);
        //rightbackDrive.setPower(0.5);
    }

    private void driveLeft(double power) {
        leftDrive.setPower(power);
        //rightDrive.setPower(0.5);
        leftbackDrive.setPower(power);
        //rightbackDrive.setPower(0.5);
    }

    private void driveRight(double power) {
        //leftDrive.setPower(0.5);
        rightDrive.setPower(power);
        //leftbackDrive.setPower(0.5);
        rightbackDrive.setPower(power);
    }

    private void move(double power) {
        driveRight(power);
        driveLeft(power);
//        rightDrive.setPower(power);
//        leftDrive.setPower(power);
//        rightbackDrive.setPower(power);
//        leftbackDrive.setPower(power);
    }
    private void turn(double power) {
        shiftRight(power);
        shiftLeft(power);
//        rightDrive.setPower(power);
//        leftDrive.setPower(power);
//        rightbackDrive.setPower(-power);
//        leftbackDrive.setPower(-power);
    }

    private void openRightGrabSide() {
        double currentGrabRightPosition = grabright.getPosition();
        double newGrabRightPosition = currentGrabRightPosition-0.01;
        grabright.setPosition(newGrabRightPosition);
    }

    private void closeRightGrabSide() {
        double currentGrabRightPosition = grabright.getPosition();
        double newGrabRightPosition = currentGrabRightPosition+0.01;
        grabright.setPosition(newGrabRightPosition);
    }

    private void closeLeftGrabSide() {
        double currentGrabLeftPosition = grableft.getPosition();
        double newGrabLeftPosition = currentGrabLeftPosition+0.01;
        grableft.setPosition(newGrabLeftPosition);
    }

    private void openLeftGrabSide() {
        double currentGrabLeftPosition = grableft.getPosition();
        double newGrabLeftPosition = currentGrabLeftPosition-0.01;
        grableft.setPosition(newGrabLeftPosition);
    }
}
