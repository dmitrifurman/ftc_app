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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


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

@TeleOp(name="turtle-bot controller", group="Linear Opmode")
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
    private Servo grableft;
    private Servo grabright;
    private static final double MAX_GRAB = 0.775;     // Maximum rotational position
    private static final double MIN_GRAB = 0.0;
    boolean isItOpen = false;
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

        elevator = hardwareMap.get(DcMotor.class, "elevator");
        grableft = hardwareMap.get(Servo.class, "grableft");
        grabright = hardwareMap.get(Servo.class, "grabright");
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

        grabright.setDirection(Servo.Direction.REVERSE);
        grableft.setDirection(Servo.Direction.FORWARD);

        Log.d("Turtle", "Init Left: "+ grableft.getPosition());
        Log.d("Turtle", "Init Right: "+ grabright.getPosition());
        Log.d("Turtle", "Init Left Controller: "+ grableft.getController().getServoPosition(0));
        Log.d("Turtle", "Init Right Controller: "+ grabright.getController().getServoPosition(1));
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = gamepad1.left_stick_y;
            double turn = gamepad1.left_stick_x;
            double spin = gamepad2.left_stick_x;
            double shift = gamepad1.right_stick_x;
            double lift = gamepad2.left_stick_y;

            boolean grab = gamepad2.a;

            if (drive > 0.5){
                leftDrive.setPower(0.5);
                rightDrive.setPower(0.5);
                leftbackDrive.setPower(0.5);
                rightbackDrive.setPower(0.5);
            } else if (drive < -0.5){
                leftDrive.setPower(-0.5);
                rightDrive.setPower(-0.5);
                leftbackDrive.setPower(-0.5);
                rightbackDrive.setPower(-0.5);
            } else {
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                leftbackDrive.setPower(0);
                rightbackDrive.setPower(0);
            }
            if (turn > 0.5){
                leftDrive.setPower(0.5);
                rightDrive.setPower(-0.5);
                leftbackDrive.setPower(0.5);
                rightbackDrive.setPower(-0.5);
            } else if (turn < -0.5){
                leftDrive.setPower(-0.5);
                rightDrive.setPower(0.5);
                leftbackDrive.setPower(-0.5);
                rightbackDrive.setPower(0.5);
            } else {
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                leftbackDrive.setPower(0);
                rightbackDrive.setPower(0);
            }

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;
//motor derection for shifting
            if (lift < 0) {
                elevator.setPower(0.5);
            } else if (lift > 0) {
                elevator.setPower(-0.15);
            } else {
                elevator.setPower(0);
            }

            if (grab && isItOpen) {
                grabright.setPosition(MIN_GRAB);
                grableft.setPosition(MIN_GRAB);
                isItOpen = false;
                Log.d("Turtle", "Grab Left: "+ grableft.getPosition());
                Log.d("Turtle", "Grab Right: "+ grabright.getPosition());
                Log.d("Turtle", "Grab Left Controller: "+ grableft.getController().getServoPosition(0));
                Log.d("Turtle", "Grab Right Controller: "+ grabright.getController().getServoPosition(1));
            } else if (grab && !isItOpen){
                grabright.setPosition(MAX_GRAB);
                grableft.setPosition(MAX_GRAB);
                isItOpen =  true;
                Log.d("Turtle", "Drop Left: "+ grableft.getPosition());
                Log.d("Turtle", "Drop Right: "+ grabright.getPosition());
                Log.d("Turtle", "Drop Left Controller: "+ grableft.getController().getServoPosition(0));
                Log.d("Turtle", "Drop Right Controller: "+ grabright.getController().getServoPosition(1));
            }

            if (spin > 0){
                spinner.setPower(0.5);
            } else if (spin < 0){
                spinner.setPower(-0.5);
            } else {
                spinner.setPower(0);
            }
            if (shift < -0.5) {
                leftDrive.setPower(-0.5);
                rightDrive.setPower(-0.5);
                leftbackDrive.setPower(0.5);
                rightbackDrive.setPower(0.5);
                //motor derection for regular movement
            } else if (shift > 0.5) {
                leftDrive.setPower(0.5);
                rightDrive.setPower(0.5);
                leftbackDrive.setPower(-0.5);
                rightbackDrive.setPower(-0.5);
            } else {
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                leftbackDrive.setPower(0);
                rightbackDrive.setPower(0);
            }



        }
    }
}
