package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by josef on 2/8/2018.
 */
@Autonomous(name = "foot test", group = "Pushbot")
@Disabled
public class footTest extends AutoBase {

    @Override
    public void executeSpecificOpMode() {
        sleep(2000);
        straitDrive(DRIVE_SPEED, 12.0);

    }
}