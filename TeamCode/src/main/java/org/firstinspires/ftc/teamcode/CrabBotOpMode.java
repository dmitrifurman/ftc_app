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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Crab Bot OpMode", group = "Linear Opmode")
public class CrabBotOpMode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor elevator = null;
    private static final double MAX_GRAB = 0.75;     // Maximum rotational position
    private static final double MIN_GRAB = 0.0;
    private static final double MAX_SPIN = 1.0;     // Maximum rotational position
    private static final double MIN_SPIN = 0.0;     // Minimum rotational position
    private Servo grabber;
    private Servo spingrabber;
    private double positionspin = (MAX_SPIN - MIN_SPIN) / 2 - .0625; // Start at halfway position
    private boolean pressedcc = false;
    private boolean pressedc = false;

    public void runOpMode() {
        initialize();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            handleElevator();

            handleGrabber();

            handleDrive();

            // Show the elapsed game time
            outputTelemetery("Status", "Run Time: " + runtime.toString());
        }
    }

    private void initialize() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        elevator = hardwareMap.get(DcMotor.class, "elevator");
        grabber = hardwareMap.get(Servo.class, "grabber");
        spingrabber = hardwareMap.get(Servo.class, "spingrabber");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        elevator.setDirection(DcMotor.Direction.FORWARD);
        grabber.setDirection(Servo.Direction.FORWARD);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
    }

    private void handleDrive() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        // Use left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        leftPower = Range.clip(drive + turn, -1.0, 1.0);
        rightPower = Range.clip(drive - turn, -1.0, 1.0);

        // Send calculated power to wheels
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        // Show the wheel power.
        outputTelemetery("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    private void handleElevator() {
        boolean drop = gamepad1.b;
        boolean rise = gamepad1.a;

        if (rise) {
            elevator.setPower(1.0);
        }
        if (drop) {
            elevator.setPower(-0.15);
        } else {
            elevator.setPower(0);
        }
    }

    private void handleGrabber() {

        boolean grab = gamepad1.x;
        boolean grabberturnclock = gamepad1.dpad_right;
        boolean grabberturncounterclock = gamepad1.dpad_left;

        //servo motor 0
        if (grab) {
            grabber.setPosition(MIN_GRAB);
        } else {
            grabber.setPosition(MAX_GRAB);
        }
        spingrabber.setPosition(positionspin);

        if (grabberturncounterclock) {
            if (!pressedcc) {
                positionspin = positionspin - .125;
                pressedcc = true;
            }
        } else {
            pressedcc = false;
        }
        if (grabberturnclock) {
            if (!pressedc) {
                positionspin = positionspin + .125;
                pressedc = true;
            }
        } else {
            pressedc = false;
        }
    }

    private void outputTelemetery(String field, Object... data) {
        telemetry.addData(field, data);
        telemetry.update();
    }
}
