package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.graphics.Color;
import android.preference.PreferenceManager;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Contains variables and methods used to control the robot.
 */
@Disabled
abstract class OpModeBase extends LinearOpMode {
    //*************** Declare Hardware Devices ***************

    //Motors
    DcMotor motorLeftFront;
    DcMotor motorLeftBack;
    DcMotor motorRightFront;
    DcMotor motorRightBack;

    //Sensors
    BNO055IMU imu;

    //SharedPreferences
    private SharedPreferences sharedPreferences;
    Direction moveDirection;

    String allianceColor;
    private String location;
    private int delay;

    //Autonomous Specific Configuration
    double moveSpeed = .65;
    double kP = 0.0;
    double ticksRatio = 1;

    double turnSpeed = .3;
    private double turnSlowdown = .1;

    Orientation angles;

    enum Direction {
        LEFT, RIGHT;

        //Taken from: http://stackoverflow.com/a/17006263
        private static OpModeBase.Direction[] vals = values();

        public OpModeBase.Direction next() {
            return vals[(this.ordinal() + 1) % vals.length];
        }
    }

    /**
     * Configures all parts of the robot.
     */
    public void runOpMode() {
        sharedPreferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);

        //*************** Map hardware devices ***************

        //Drive Motors
        motorLeftFront = hardwareMap.dcMotor.get("left_front");
        motorLeftBack = hardwareMap.dcMotor.get("left_back");
        motorRightFront = hardwareMap.dcMotor.get("right_front");
        motorRightBack = hardwareMap.dcMotor.get("right_back");

        //*************** Configure hardware devices ***************

        //Motors

        //Drive motors
        motorLeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Autonomous methods that need RUN_TO_POSITION will set the motors, RUN_USING_ENCODER is required for teleop and gyro turn
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //*************** Configure SharedPreferences ***************
        allianceColor = sharedPreferences.getString("com.qualcomm.ftcrobotcontroller.Autonomous.Color", "null");
        location = sharedPreferences.getString("com.qualcomm.ftcrobotcontroller.Autonomous.Location", "null");
        delay = sharedPreferences.getInt("com.qualcomm.ftcrobotcontroller.Autonomous.Delay", 0);

        if (allianceColor.equals("Blue")) {
            moveDirection = Direction.RIGHT;
        } else {
            moveDirection = Direction.LEFT;
        }

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        telemetry.addData("Ready to start program", "");
        telemetry.addData("Alliance color", allianceColor);
        telemetry.addData("Encoders", motorLeftFront.getCurrentPosition() + motorLeftBack.getCurrentPosition() + motorRightFront.getCurrentPosition() + motorRightBack.getCurrentPosition());
        telemetry.update();
    }

    /**
     * Calls turn() with a default of recurse = 1, maxSpeed = turnSpeed
     * It recursively calls itself to improve accuracy and fix overshooting.
     * The speed decreases as the turn nears completion.
     *
     * @param degrees number of degrees to turn (positive)
     * @param direction Enum Direction.RIGHT or Direction.LEFT
     * @see OpModeBase#turn(double, OpModeBase.Direction, double, int)
     * @see ModernRoboticsI2cGyro
     * @see DcMotor
     */
    void turn(double degrees, OpModeBase.Direction direction) {
        if (!opModeIsActive()) return;
        turn(degrees, direction, turnSpeed, 1);
    }

    /**
     * Calls turn() with a default of recursing once
     * It recursively calls itself to improve accuracy and fix overshooting.
     * The speed decreases as the turn nears completion.
     *
     * @param degrees number of degrees to turn (positive)
     * @param direction Enum Direction.RIGHT or Direction.LEFT
     * @param maxSpeed the maximum speed of the robot
     * @see OpModeBase#turn(double, OpModeBase.Direction, double, int)
     * @see ModernRoboticsI2cGyro
     * @see DcMotor
     */
    void turn(double degrees, OpModeBase.Direction direction, double maxSpeed) {
        if(!opModeIsActive()) return;
        turn(degrees, direction, maxSpeed, 1);
    }

    /**
     * This method is used to turn the robot a specific number of degrees in a specific direction.
     * It recursively calls itself to improve accuracy and fix overshooting.
     * The speed decreases as the turn nears completion.
     *
     * @param degrees number of degrees to turn (positive)
     * @param direction Enum Direction.RIGHT or Direction.LEFT
     * @param maxSpeed the maximum speed of the robot
     * @param count the number of times to recurese
     * @see OpModeBase#turn(double, OpModeBase.Direction, double)
     * @see ModernRoboticsI2cGyro
     * @see DcMotor
     */
    void turn(double degrees, OpModeBase.Direction direction, double maxSpeed, int count) {
        if (!opModeIsActive()) return;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (direction.equals(OpModeBase.Direction.RIGHT)) degrees *= -1; //Negative degree for turning right
        double targetHeading = angles.firstAngle + degrees; //Turns are relative to current position

        //Change mode because turn() uses motor power and not motor position
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (degrees < 0) {
            while (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > targetHeading && opModeIsActive()) {
                float heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

                motorLeftFront.setPower(Range.clip(maxSpeed * (Math.abs(heading - targetHeading) / Math.abs(degrees)), turnSlowdown, maxSpeed));
                motorLeftBack.setPower(Range.clip(maxSpeed * (Math.abs(heading - targetHeading) / Math.abs(degrees)), turnSlowdown, maxSpeed));
                motorRightFront.setPower(Range.clip(-maxSpeed * (Math.abs(heading - targetHeading) / Math.abs(degrees)), -maxSpeed, -turnSlowdown));
                motorRightBack.setPower(Range.clip(-maxSpeed * (Math.abs(heading - targetHeading) / Math.abs(degrees)), -maxSpeed, -turnSlowdown));

                telemetry.addData("Distance to turn: ", Math.abs(heading - targetHeading));
                telemetry.addData("Target", targetHeading);
                telemetry.addData("Heading", heading);
                telemetry.update();
                idle();
            }
        } else { //Left
            while (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < targetHeading && opModeIsActive()) {
                float heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

                motorLeftFront.setPower(Range.clip(-maxSpeed * (Math.abs(heading - targetHeading) / Math.abs(degrees)), -maxSpeed, -turnSlowdown));
                motorLeftBack.setPower(Range.clip(-maxSpeed * (Math.abs(heading - targetHeading) / Math.abs(degrees)), -maxSpeed, -turnSlowdown));
                motorRightFront.setPower(Range.clip(maxSpeed * (Math.abs(heading - targetHeading) / Math.abs(degrees)), turnSlowdown, maxSpeed));
                motorRightBack.setPower(Range.clip(maxSpeed * (Math.abs(heading - targetHeading) / Math.abs(degrees)), turnSlowdown, maxSpeed));

                telemetry.addData("Distance to turn: ", Math.abs(heading - targetHeading));
                telemetry.addData("Target", targetHeading);
                telemetry.addData("Heading", heading);
                telemetry.update();
                idle();
            }
        }
        motorLeftFront.setPower(0);
        motorLeftBack.setPower(0);
        motorRightFront.setPower(0);
        motorRightBack.setPower(0);
        sleep(200);

        telemetry.addData("Distance to turn", Math.abs(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - targetHeading));
        telemetry.addData("Direction", -1 * (int) Math.signum(degrees));
        telemetry.update();

        if (Math.abs(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - targetHeading) > 0 && count > 0) {
            //Recurse to correct turn
            turn(Math.abs(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - targetHeading), direction.next(), .1, --count);
        }
    }

    /**
     * Calls move with a default of maxSpeed = moveSpeed, recurse = true, PID = true
     * a and b tuned for 48 inches
     * @param distance the distance in inches to move
     * @see OpModeBase#move(double, double, boolean)
     * @see DcMotor
     * @see ModernRoboticsI2cGyro
     */
    void move(double distance) {
        move(distance, moveSpeed, true, kP);
    }

    /**
     * Calls move with a default of recurse = true, PID = true
     * a and b tuned for 48 inches
     * @param distance the distance in inches to move
     * @param maxSpeed the maximum speed to run the robot
     * @see OpModeBase#move(double, double, boolean, double)
     * @see DcMotor
     * @see ModernRoboticsI2cGyro
     */
    void move(double distance, double maxSpeed) {
        move(distance, maxSpeed, true, kP);
    }

    /**
     * Calls move with a default of a = .8, b = .2
     * a and b tuned for 48 inches
     * @param distance the distance in inches to move
     * @param maxSpeed the maximum speed to run the robot
     * @param recurse whether or not to call turn()
     * @see OpModeBase#move(double, double, boolean, double)
     * @see DcMotor
     * @see ModernRoboticsI2cGyro
     */
    void move(double distance, double maxSpeed, boolean recurse) {
        move(distance, maxSpeed, recurse, kP);
    }

    /**
     * Moves the robot a specified number of inches.
     * The speed of the robot decreases in a sigmoid fashion as the target is approached.
     * It uses the gyroscope to keep the movement straight.
     * It will call turn() at the end to correct heading eror.
     *
     * @param distance the distance in inches to move
     * @param maxSpeed the maximum speed to run the robot
     * @param recurse whether or not to call turn()
     * @see OpModeBase#move(double, double)
     * @see OpModeBase#turn
     * @see DcMotor
     * @see ModernRoboticsI2cGyro
     */
    void move(double distance, double maxSpeed, boolean recurse, double kP) {
        double distanceSign = Math.signum(distance); //Necessary for moving backwards
        distance *= ticksRatio;
        float initialHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        //Change mode because move() uses setTargetPosition()
        motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLeftFront.setTargetPosition((int) (motorLeftFront.getCurrentPosition() + distance));
        motorLeftBack.setTargetPosition((int) (motorLeftBack.getCurrentPosition() + distance));
        motorRightFront.setTargetPosition((int) (motorRightFront.getCurrentPosition() + distance));
        motorRightBack.setTargetPosition((int) (motorRightBack.getCurrentPosition() + distance));

        motorLeftFront.setPower(maxSpeed);
        motorLeftBack.setPower(maxSpeed);
        motorRightFront.setPower(maxSpeed);
        motorRightBack.setPower(maxSpeed);

        while ((motorLeftFront.isBusy() && motorLeftBack.isBusy() && motorRightFront.isBusy() && motorRightBack.isBusy()) && opModeIsActive()) {
            //Only one encoder target must be reached
            double turnError = initialHeading - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            double headingError = turnError * kP * distanceSign;
            double leftPower = Range.clip(maxSpeed - headingError, .1, 1);
            double rightPower = Range.clip(maxSpeed + headingError, .1, 1);

            motorLeftFront.setPower(leftPower);
            motorLeftBack.setPower(leftPower);
            motorRightFront.setPower(rightPower);
            motorRightBack.setPower(rightPower);

            telemetry.addData("Left motor power", leftPower);
            telemetry.addData("Right motor power", rightPower);
            telemetry.addData("Current Heading", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.addData("Target Heading", initialHeading);
            telemetry.addData("Heading Error", headingError);
            telemetry.addData("Position", motorLeftFront.getCurrentPosition());
            telemetry.addData("Target", motorLeftFront.getTargetPosition());

            telemetry.update();
        }

        motorLeftFront.setPower(0);
        motorLeftBack.setPower(0);
        motorRightFront.setPower(0);
        motorRightBack.setPower(0);
        sleep(400);

        //Correct if robot turned during movement
        if (Math.abs(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - initialHeading) > 0 && recurse) {
            turn(Math.abs(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - initialHeading), imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > initialHeading ? OpModeBase.Direction.RIGHT : OpModeBase.Direction.LEFT, .1);
        }
    }
}