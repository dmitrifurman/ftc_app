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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "blue bottom", group = "Pushbot")
//@Disabled
public class blue_bottom extends AutoBase {

    @Override
    public void executeSpecificOpMode() {
        robot.spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.spinner.setPower(0);
        robot.colorHolder.setPosition(robot.MAX_GRAB);
        grab();
        sleep(1000);
        lift(0.5);
        sleep(1000);
        stopElevator();
        robot.colorHolder.setPosition(robot.MIN_SERVO);
        sleep(1000);
        blueColorArm();
        sleep(1000);
        double boxAngle = 0;
        switch (readRelic()) {
            case UNKNOWN:
                telemetry.addData("picture: ", "unknown");
                boxAngle = 113.0;
                break;
            case LEFT:
                telemetry.addData("picture: ", "left");
                boxAngle = 110.0;
                break;
            case CENTER:
                telemetry.addData("picture: ", "middle");
                boxAngle = 102.5;
                break;
            case RIGHT:
                telemetry.addData("picture: ", "right");
                boxAngle = 95.0;
                break;
        }
      /*  spin(TURN_SPEED, -180);
        sleep(1000);
        robot.colorHolder.setPosition(robot.MID_SERVO);
        sleep(1000);
        robot.colorHolder.setPosition(robot.MIN_SERVO);
        sleep(250);
        if(robot.isRed){
            gyroTurn( TURN_SPEED,  45.0);
            sleep(250);
            gyroTurn( TURN_SPEED,  -45.0);
        } else if (robot.isBlue){
            gyroTurn( TURN_SPEED,  -45.0);
            sleep(250);
            gyroTurn( TURN_SPEED,  45.0);
        }
        sleep(250);*/
        sleep(1000);
        //    robot.colorHolder.setPosition(robot.MID_SERVO);
        // sleep(1000);
        straitDrive(DRIVE_SPEED, -35.0);
        // Drive FWD 48 inches

        sleep(1000);

        //spin(-0.5, 0.25);
        //sleep(1000);
        //gyroDrive(DRIVE_SPEED, 12.0, 0.0);    // Drive FWD 48 inches
        //angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        //gyroDrive(DRIVE_SPEED, -12, 0.0);
        // Turn  CW  to  45 Degrees

        // Hold  45 Deg heading for a 1/2 secon
        gyroTurn(TURN_SPEED, boxAngle);
        // Drive REV 48 inches
        sleep(1000);
        relese(2.0);
        sleep(1000);
        straitDrive(DRIVE_SPEED, -12.0);

        sleep(1000);
        //sleep(1000);
        drop();
        sleep(1000);

        straitDrive(DRIVE_SPEED, 7.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}
