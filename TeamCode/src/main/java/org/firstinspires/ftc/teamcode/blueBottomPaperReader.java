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

@Autonomous(name = "blue bottom + picture reader", group = "Pushbot")
//@Disabled
public class blueBottomPaperReader extends AutoBase {

    @Override
    public void executeSpecificOpMode() {

        blueColorArm();
        sleep(1000);
      double shiftDistance = 0.0;

        switch (readRelic()) {
            case UNKNOWN:
                log("unknown");
                shiftDistance = -28.0;
                break;
            case LEFT:
                log("left");
                shiftDistance = -24.0;
                break;
            case CENTER:
                log("middle");
                shiftDistance = -28.0;
                break;
            case RIGHT:
                log("right");
                shiftDistance = -36.0;
                break;
        }


        sleep(500);
        //    robot.colorHolder.setPosition(robot.MID_SERVO);
        // sleep(1000);

        straitDrive(DRIVE_SPEED, shiftDistance);
        sleep(500);


        gyroTurn(TURN_SPEED, 90.25);
        // Drive REV 48 inches
        sleep(500);
        relese(2.0);
        sleep(500);
        straitDrive(DRIVE_SPEED, -15.0);

        sleep(500);
        //sleep(1000);
        drop();
        sleep(500);

        straitDrive(DRIVE_SPEED, 8.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}
