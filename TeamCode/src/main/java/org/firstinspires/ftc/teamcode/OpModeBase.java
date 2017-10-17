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
import com.qualcomm.robotcore.hardware.DistanceSensor;
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
    DcMotor motorLeft1;
    DcMotor motorLeft2;
    DcMotor motorRight1;
    DcMotor motorRight2;

    //Sensors
    BNO055IMU imu;
    ColorSensor colorSensor;
    DistanceSensor distanceSensor;

    //SharedPreferences
    private SharedPreferences sharedPreferences;
    Direction moveDirection;

    String allianceColor;
    private String location;
    private int delay;

    //Autonomous Specific Configuration
    double moveSpeed = .65;
    double kP = 0.0;
    double ticksRatio = 1; //Ticks / inch

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
        motorLeft1 = hardwareMap.dcMotor.get("left 1");
        motorLeft2 = hardwareMap.dcMotor.get("left 2");
        motorRight1 = hardwareMap.dcMotor.get("right 1");
        motorRight2 = hardwareMap.dcMotor.get("right 2");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        colorSensor = hardwareMap.colorSensor.get("color distance");
        colorSensor.enableLed(true);
        distanceSensor = hardwareMap.get(DistanceSensor.class, "color distance");

        //*************** Configure hardware devices ***************

        //Motors

        //Drive motors
        motorLeft1.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLeft2.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRight1.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRight2.setDirection(DcMotorSimple.Direction.FORWARD);

        motorLeft1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft1.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Autonomous methods that need RUN_TO_POSITION will set the motors, RUN_USING_ENCODER is required for TeleOp and gyro turn
        motorLeft2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLeft1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //Default is float
        motorLeft2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        telemetry.addData("Ready to start program", "");
        telemetry.addData("Alliance color", allianceColor);
        telemetry.addData("Encoders", motorLeft1.getCurrentPosition() + motorLeft2.getCurrentPosition() + motorRight1.getCurrentPosition() + motorRight2.getCurrentPosition());
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
        motorLeft1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeft2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (degrees < 0) {
            while (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > targetHeading && opModeIsActive()) {
                float heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

                motorLeft1.setPower(Range.clip(maxSpeed * (Math.abs(heading - targetHeading) / Math.abs(degrees)), turnSlowdown, maxSpeed));
                motorLeft2.setPower(Range.clip(maxSpeed * (Math.abs(heading - targetHeading) / Math.abs(degrees)), turnSlowdown, maxSpeed));
                motorRight1.setPower(Range.clip(-maxSpeed * (Math.abs(heading - targetHeading) / Math.abs(degrees)), -maxSpeed, -turnSlowdown));
                motorRight2.setPower(Range.clip(-maxSpeed * (Math.abs(heading - targetHeading) / Math.abs(degrees)), -maxSpeed, -turnSlowdown));

                telemetry.addData("Distance to turn: ", Math.abs(heading - targetHeading));
                telemetry.addData("Target", targetHeading);
                telemetry.addData("Heading", heading);
                telemetry.update();
                idle();
            }
        } else { //Left
            while (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < targetHeading && opModeIsActive()) {
                float heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

                motorLeft1.setPower(Range.clip(-maxSpeed * (Math.abs(heading - targetHeading) / Math.abs(degrees)), -maxSpeed, -turnSlowdown));
                motorLeft2.setPower(Range.clip(-maxSpeed * (Math.abs(heading - targetHeading) / Math.abs(degrees)), -maxSpeed, -turnSlowdown));
                motorRight1.setPower(Range.clip(maxSpeed * (Math.abs(heading - targetHeading) / Math.abs(degrees)), turnSlowdown, maxSpeed));
                motorRight2.setPower(Range.clip(maxSpeed * (Math.abs(heading - targetHeading) / Math.abs(degrees)), turnSlowdown, maxSpeed));

                telemetry.addData("Distance to turn: ", Math.abs(heading - targetHeading));
                telemetry.addData("Target", targetHeading);
                telemetry.addData("Heading", heading);
                telemetry.update();
                idle();
            }
        }
        motorLeft1.setPower(0);
        motorLeft2.setPower(0);
        motorRight1.setPower(0);
        motorRight2.setPower(0);
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
        motorLeft1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLeft1.setTargetPosition((int) (motorLeft1.getCurrentPosition() + distance));
        motorLeft2.setTargetPosition((int) (motorLeft2.getCurrentPosition() + distance));
        motorRight1.setTargetPosition((int) (motorRight1.getCurrentPosition() + distance));
        motorRight2.setTargetPosition((int) (motorRight2.getCurrentPosition() + distance));

        motorLeft1.setPower(maxSpeed);
        motorLeft2.setPower(maxSpeed);
        motorRight1.setPower(maxSpeed);
        motorRight2.setPower(maxSpeed);

        while ((motorLeft1.isBusy() && motorLeft2.isBusy() && motorRight1.isBusy() && motorRight2.isBusy()) && opModeIsActive()) {
            //Only one encoder target must be reached
            double turnError = initialHeading - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            double headingError = turnError * kP * distanceSign;
            double leftPower = Range.clip(maxSpeed - headingError, .1, 1);
            double rightPower = Range.clip(maxSpeed + headingError, .1, 1);

            motorLeft1.setPower(leftPower);
            motorLeft2.setPower(leftPower);
            motorRight1.setPower(rightPower);
            motorRight2.setPower(rightPower);

            telemetry.addData("Left motor power", leftPower);
            telemetry.addData("Right motor power", rightPower);
            telemetry.addData("Current Heading", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.addData("Target Heading", initialHeading);
            telemetry.addData("Heading Error", headingError);
            telemetry.addData("Position", motorLeft1.getCurrentPosition());
            telemetry.addData("Target", motorLeft1.getTargetPosition());

            telemetry.update();
        }

        motorLeft1.setPower(0);
        motorLeft2.setPower(0);
        motorRight1.setPower(0);
        motorRight2.setPower(0);
        sleep(400);

        //Correct if robot turned during movement
        if (Math.abs(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - initialHeading) > 0 && recurse) {
            turn(Math.abs(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - initialHeading), imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > initialHeading ? OpModeBase.Direction.RIGHT : OpModeBase.Direction.LEFT, .1);
        }
    }

    int getColor() {
        float[] hsv = {0F, 0F, 0F};

        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsv);

        if ((hsv[0] < 30 || hsv[0] > 340) && hsv[1] > .2) return Color.RED;
        else if ((hsv[0] > 170 && hsv[0] < 260) && hsv[1] > .2) return Color.BLUE;
        return 0;
    }
}