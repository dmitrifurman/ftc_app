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


@Autonomous(name="blue top", group="Pushbot")
//@Disabled
public class blue_top extends AutoBase {

    @Override
    public void executeSpecificOpMode() {
        blueColorArm();
        sleep(1000);
 /*       double boxAngle = 0;
        switch (readRelic()) {
            case UNKNOWN:
                log("unknown");
                boxAngle = 0.0;
                break;
            case LEFT:
                log("left");
                boxAngle = 46.0;
                break;
            case CENTER:
                log("middle");
                boxAngle = 23.0;
                break;
            case RIGHT:
                log("right");
                boxAngle = 0.0;
                break;
        }

        //spin(-0.5, 1);
        spin(TURN_SPEED, -180);
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
        straitDrive(DRIVE_SPEED, -24.0);
        sleep(1000);
   //     shiftDrive(DRIVE_SPEED, -6.0);
     //   sleep(500);
        gyroTurn(TURN_SPEED, -9.5);
        sleep(500);
        relese(2.0);
        sleep(500);
        //sleep(1000);

        sleep(500);
        straitDrive(DRIVE_SPEED, -20.0);

        sleep(500);
        drop();

sleep(500);
       straitDrive(DRIVE_SPEED, 5.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


}
