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

/**
 * This file illustrates the concept of driving a path based on Gyro heading and encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that you have a Modern Robotics I2C gyro with the name "gyro"
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *  This code requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 *  In order to calibrate the Gyro correctly, the robot must remain stationary during calibration.
 *  This is performed when the INIT button is pressed on the Driver Station.
 *  This code assumes that the robot is stationary when the INIT button is pressed.
 *  If this is not the case, then the INIT should be performed again.
 *
 *  Note: in this example, all angles are referenced to the initial coordinate frame set during the
 *  the Gyro Calibration process, or whenever the program issues a resetZAxisIntegrator() call on the Gyro.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clock Wise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="red bottom + picture reader", group="Pushbot")
//@Disabled
public class redBottomColorReader extends AutoBase {

    @Override
    public void executeSpecificOpMode() {
        robot.spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.spinner.setPower(0);
        robot.colorHolder.setPosition(robot.MAX_GRAB);
        grab();
        sleep(500);
        lift(0.5);
        sleep(500);
        stopElevator();
        robot.colorHolder.setPosition(robot.MIN_SERVO);
        sleep(1000);

  redColorArm();



       //gyroTurn(TURN_SPEED, 180);
  //      straitDrive(DRIVE_SPEED, -3.0);
        sleep(1000);

        double shiftDistance = 0;
        double boxAngle = 0;
        switch (readRelic()) {
            case UNKNOWN:
                shiftDistance = 23.0;
                boxAngle = 154.5;
                break;
            case LEFT:
                shiftDistance = 18.0;
                boxAngle =152.5;
                break;
            case CENTER:
                shiftDistance = 23.0;
                boxAngle =154.5;
                break;
            case RIGHT:
                shiftDistance = 28.0;
                boxAngle = 154.5;
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
        sleep(500);
        //    robot.colorHolder.setPosition(robot.MID_SERVO);
        // sleep(1000);
        straitDrive(DRIVE_SPEED, shiftDistance);
        // Drive FWD 48 inches

     sleep(500);

        //spin(-0.5, 0.25);
        //sleep(1000);
        //gyroDrive(DRIVE_SPEED, 12.0, 0.0);    // Drive FWD 48 inches
        //angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        //gyroDrive(DRIVE_SPEED, -12, 0.0);
        // Turn  CW  to  45 Degrees

        // Hold  45 Deg heading for a 1/2 secon
        gyroTurn(TURN_SPEED, boxAngle);
        // Drive REV 48 inches
        sleep(500);
        relese(2.0);
        sleep(500);
        straitDrive(DRIVE_SPEED, -12.0);

        sleep(500);
        //sleep(1000);
        drop();
        sleep(500);

        straitDrive(DRIVE_SPEED, 5.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}
