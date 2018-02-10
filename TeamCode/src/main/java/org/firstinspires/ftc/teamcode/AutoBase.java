package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Locale;

/**
 * Created by josef on 1/13/2018.
 */

public abstract class AutoBase extends LinearOpMode {
    //start at ~25'
    /* Declare OpMode members. */
    MyHardwarePushbot robot = new MyHardwarePushbot();   // Use a Pushbot's hardware
    // The IMU sensor object
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;


    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;

    private ElapsedTime runtime = new ElapsedTime();


    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    //static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
    //                                                  (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double COUNTS_PER_INCH = 87;
    static final double COUNTS_PER_SHIFT = 108;
    static final int SPIN_TOILET_PART = 560;
    protected static final String TAG = "gyro";
    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double DRIVE_SPEED = 0.35;     // Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.175;     // Nominal half speed for better accuracy.
    boolean isRed;
    boolean isBlue;
    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable

    abstract public void executeSpecificOpMode();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        initRelic();
        initGyro();

        // reset motors and turn on encoders.
        robot.resetMotors();
        robot.setRunUsingEncoder();

        waitForStart();

        robot.spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.spinner.setPower(0);

        robot.colorHolder.setPosition(robot.MIN_GRAB);
        sleep(250);
        grab();
        sleep(500);
        lift(0.5);
        sleep(500);
        stopElevator();
        sleep(1000);
        executeSpecificOpMode();
    }

    private void initGyro() {
        //gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set up our telemetry dashboard
        composeTelemetry();

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        telemetry.addData(">", "Calibrating Gyro");
        telemetry.addData("is calibrated", imu.isGyroCalibrated());
        telemetry.update();
    }

    public void stopElevator() {
        robot.elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void readColor() {

        NormalizedRGBA colors = robot.colorSensor.getNormalizedColors();
      // rightClaw.setPosition(MID_SERVO);

        isRed = colors.red > colors.blue*1.33 & colors.red > colors.green*1.33;
        isBlue = colors.blue > colors.red;

        telemetry.addData("read color is blue", isBlue);
        telemetry.update();
    }

    public void sawRed() {
       log("saw red");
       gyroTurn(TURN_SPEED, 21);
        sleep(100);
       robot.colorHolder.setPosition(robot.MAX_GRAB);
    }

    public void sawBlue() {
        log("saw blue");
        gyroTurn(TURN_SPEED, -21);
        sleep(100);
        robot.colorHolder.setPosition(robot.MAX_GRAB);
        sleep(100);
        log("saw blue2");
        straitDrive(0, 0);
        sleep(100);
        gyroTurn(TURN_SPEED, 21);
        log("saw blue3");
    }
    public void noColor(){
        robot.colorHolder.setPosition(robot.MAX_GRAB);
        sleep(250);
        gyroTurn(TURN_SPEED, 21);
    }
    public void blueColorArm() {

        readColor();

        if (isBlue) {
            sawBlue();
        } else {
           sawRed();
        }// else {
          //  noColor();
       // }
   }

    public void redColorArm() {
        readColor();
        if (isBlue) {
            sawRed();
        } else {
            sawBlue();
    }
}
    public void shiftDrive(double speed,
                           double distance) {
        robot.leftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        robot.rightDrive.setDirection(DcMotor.Direction.FORWARD);
        robot.rightbackDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        robot.leftbackDrive.setDirection(DcMotor.Direction.REVERSE);
        gyroDrive(speed, distance, 0, COUNTS_PER_SHIFT);
    }


    /**
     * Method to drive on a fixed compass bearing (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the opmode running.
     * <p>
     * //* @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * // * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * //* @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     * 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     * If a relative angle is required, add/subtract from current heading.
     */
    public void straitDrive(double speed,
                            double distance) {
        robot.leftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        robot.rightDrive.setDirection(DcMotor.Direction.FORWARD);
        robot.rightbackDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        robot.leftbackDrive.setDirection(DcMotor.Direction.REVERSE);
        gyroDrive(speed, distance, 0, COUNTS_PER_INCH);
    }

    public void gyroDrive(double speed,
                          double distance,
                          double angle, double countsPerInch) {

        int newLeftTarget;
        int newRightTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            int spinCounts;

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * COUNTS_PER_INCH);

            newLeftTarget = robot.leftDrive.getCurrentPosition() + moveCounts;
            newRightTarget = robot.rightDrive.getCurrentPosition() + moveCounts;
            int newLeftbackTarget = robot.leftbackDrive.getCurrentPosition() + moveCounts;
            int newRightbackTarget = robot.rightbackDrive.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);
            robot.leftbackDrive.setTargetPosition(newLeftbackTarget);
            robot.rightbackDrive.setTargetPosition(newRightbackTarget);

            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftbackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightbackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftDrive.setPower(speed);
            robot.rightDrive.setPower(speed);
            robot.leftbackDrive.setPower(Math.abs(speed));
            robot.rightbackDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy() && robot.rightbackDrive.isBusy() && robot.rightbackDrive.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.leftDrive.setPower(leftSpeed);
                robot.rightDrive.setPower(rightSpeed);
                robot.leftbackDrive.setPower(leftSpeed);
                robot.rightbackDrive.setPower(rightSpeed);


                // Display drive status for the driver.
    /*            telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Actual", "%7d:%7d", robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                telemetry.update();
            }
*/
            }          //stop motion and reset encoder
            robot.stopMotion();
            robot.setRunUsingEncoder();
        }
    }

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     * <p>
     * <p>
     * 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     * If a relative angle is required, add/subtract from current heading.
     */
    public void grab() {
        robot.grableft.setPosition(robot.MID_SERVO);
        robot.grabright.setPosition(robot.MID_SERVO);
    }

    public void drop() {
        robot.grableft.setPosition(robot.MAX_GRAB);
        robot.grabright.setPosition(robot.MAX_GRAB);
    }


    public void gyroTurn(double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.addData("first angle", angles.firstAngle);
            telemetry.addData("second angle", angles.secondAngle);
            telemetry.addData("third angle", angles.thirdAngle);
            telemetry.update();
        }

        //stop motion and reset encoder
        robot.resetMotors();
    }

    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     *
     * @param speed    Desired speed of turn.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     * @param holdTime Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold(double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.stopMotion();
    }

    public void lift(double holdtime) {
        robot.elevator.setPower(0.75);
    }

    public void relese(double holdtime) {
        robot.elevator.setPower(-0.05);
    }

    /**
     * Perform one cycle of closed loop heading control.
     * <p>
     * //@param speed     Desired speed of turn.
     * //param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     * 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     * If a relative angle is required, add/subtract from current heading.
     * // * @param PCoeff    Proportional Gain coefficient
     *
     * @return
     */
    public void spin(double speed, int holdTime) {
        robot.spinner.setPower(speed);
        ElapsedTime holdTimer = new ElapsedTime();
        log("spin");
        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            robot.spinner.setPower(speed);
            log("it works");
        }

    }

    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        //why can't this a double
        if(error <= .5 ) {
            //this should give us more accurate angle
            
        }
        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.leftDrive.setPower(leftSpeed);
        robot.rightDrive.setPower(rightSpeed);
        robot.leftbackDrive.setPower(leftSpeed);
        robot.rightbackDrive.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        if (targetAngle == 0) return 0;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - angles.firstAngle;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
            }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override
                    public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override

                    public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel * gravity.xAccel
                                        + gravity.yAccel * gravity.yAccel
                                        + gravity.zAccel * gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    protected void log(String message, Throwable... throwables) {
        if (throwables.length > 0) {
            Log.d(TAG, message, throwables[0]);
        } else {
            Log.d(TAG, message);
        }
        telemetry.addLine(TAG + ": " + message);
        telemetry.update();
    }

    private void initRelic() {
                /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "AZHX20r/////AAAAGd1blCVgEEF4pimwL/e5T49yqjI5BT9bFjgpURzJdG3AjWY3aNplVb9+OZtMMaloqkHQaoRg6VP2xLcG6++SLFTLs5Kss4LLXhwHLLgBS3FNpz6gtybU9+IXgrjzmIEdasBxKDlb8PVWy//nBIwOpHoMz221siALSRpw/nXVvdZkc6fURYVcSvNB7iwaPSwShL/Cn0hOnJB2JjnBT7I1NACYyRXuEurrkWvlIdVnWjirSybjqHq5OcEq3M7juns02EypjslYiYnYe+Nhp6RtYbS6iArMFQ3gWcrFHI3JXotvneQPe8S4JSqGjRvER3t9FdTnXdiGMq0MdJDFf0yQRf/k/PP9L83zL/VxFmAUaVC+";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
    }

    public RelicRecoveryVuMark readRelic() {
        relicTrackables.activate();

        RelicRecoveryVuMark vuMark = null;
        runtime.reset();
        do {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
        }
        while ((opModeIsActive() && (runtime.seconds() < 3)) && vuMark == RelicRecoveryVuMark.UNKNOWN);

        /**
         * See if any of the instances of {@link relicTemplate} are currently visible.
         * {@link RelicRecoveryVuMark} is an enum which can have the following values:
         * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
         * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
         */

        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
            telemetry.addData("VuMark", "%s visible", vuMark);
        } else {
            telemetry.addData("VuMark", "not visible");
        }

        telemetry.update();
        return vuMark;
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }


}
