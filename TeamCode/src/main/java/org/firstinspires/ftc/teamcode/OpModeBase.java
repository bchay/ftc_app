package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.graphics.Color;
import android.preference.PreferenceManager;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

abstract class OpModeBase extends LinearOpMode {
    //*************** Declare Hardware Devices ***************

    //Motors
    DcMotor motorLeftFront;
    DcMotor motorLeftBack;
    DcMotor motorRightFront;
    DcMotor motorRightBack;

    DcMotor shooter;
    DcMotor intake;

    //Servos
    Servo buttonPresser;
    Servo ballStop;

    //Sensors
    private ModernRoboticsI2cGyro gyro;
    private ColorSensor colorSensor;
    private OpticalDistanceSensor ods;

    //Variables
    final double BUTTON_PRESSER_IN = .2;
    final double BUTTON_PRESSER_OUT = 1;

    final double BALL_STOP_UP = 0;
    final double BALL_STOP_BLOCKED = .63;

    //SharedPreferences
    SharedPreferences sharedPreferences;
    Direction moveDirection;

    String allianceColor;
    String location;
    int delay;

    //Autonomous Specific Configuration
    double moveSpeed = .85;
    double turnSpeed = .3;
    private double kP = .04;
    double slowdownMin = .2;

    private double odsWhite = .51; //Value from ods.getRawLightDetected()
    private double odsGray = .07;
    private double odsEdge = .25;

    enum Direction {
        LEFT, RIGHT;

        //Taken from: http://stackoverflow.com/a/17006263
        private static OpModeBase.Direction[] vals = values();

        public OpModeBase.Direction next() {
            return vals[(this.ordinal() + 1) % vals.length];
        }
    }

    //Teleop specific configuration
    double motorMax = 1;

    //Initialize all hardware, do setup for opmodes
    public void runOpMode() {
        sharedPreferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);

        //*************** Map hardware devices ***************

        //Drive Motors
        motorLeftFront = hardwareMap.dcMotor.get("left_front");
        motorLeftBack = hardwareMap.dcMotor.get("left_back");
        motorRightFront = hardwareMap.dcMotor.get("right_front");
        motorRightBack = hardwareMap.dcMotor.get("right_back");

        shooter = hardwareMap.dcMotor.get("shooter");
        intake = hardwareMap.dcMotor.get("intake");

        //Button Servos
        buttonPresser = hardwareMap.servo.get("button_presser");
        ballStop = hardwareMap.servo.get("ball_stop");

        //Sensor Declaration
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        colorSensor = hardwareMap.colorSensor.get("color");
        ods = hardwareMap.opticalDistanceSensor.get("ods");

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

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        //Servos
        buttonPresser.setPosition(BUTTON_PRESSER_IN);
        ballStop.setPosition(BALL_STOP_BLOCKED);

        //Sensors

        //Gyro
        gyro.calibrate();

        //Wait while gyro is calibrating
        while (gyro.isCalibrating() && !isStopRequested()) {
            sleep(50);
            idle();
        }

        //Color Sensor
        colorSensor.enableLed(false);

        //*************** Configure SharedPreferences ***************
        allianceColor = sharedPreferences.getString("com.qualcomm.ftcrobotcontroller.Autonomous.Color", "null");
        location = sharedPreferences.getString("com.qualcomm.ftcrobotcontroller.Autonomous.Location", "null");
        delay = sharedPreferences.getInt("com.qualcomm.ftcrobotcontroller.Autonomous.Delay", 0);

        if (allianceColor.equals("Blue")) {
            moveDirection = Direction.RIGHT;
        } else {
            moveDirection = Direction.LEFT;
        }

        telemetry.addData("Ready to start program", "");
        telemetry.addData("Alliance color", allianceColor);
        telemetry.update();
    }

    void turn(double degrees, OpModeBase.Direction direction, double maxSpeed) { //count is optional, set to 0 if not provided
        if (!opModeIsActive()) return;
        turn(degrees, direction, maxSpeed, 0);
    }

    void turn(double degrees, OpModeBase.Direction direction, double maxSpeed, int count) {
        if (!opModeIsActive()) return;
        if (direction.equals(OpModeBase.Direction.RIGHT)) degrees *= -1; //Negative degree for turning right
        double targetHeading = gyro.getIntegratedZValue() + degrees;

        //Change mode because turn() uses motor power and not motor position
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (degrees < 0) {
            while (gyro.getIntegratedZValue() > targetHeading && opModeIsActive()) {
                motorLeftFront.setPower(Range.clip(maxSpeed * (Math.abs(gyro.getIntegratedZValue() - targetHeading) / Math.abs(degrees)), .1, maxSpeed));
                motorLeftBack.setPower(Range.clip(maxSpeed * (Math.abs(gyro.getIntegratedZValue() - targetHeading) / Math.abs(degrees)), .1, maxSpeed));
                motorRightFront.setPower(Range.clip(-maxSpeed * (Math.abs(gyro.getIntegratedZValue() - targetHeading) / Math.abs(degrees)), -.1, -.2));
                motorRightBack.setPower(Range.clip(-maxSpeed * (Math.abs(gyro.getIntegratedZValue() - targetHeading) / Math.abs(degrees)), -.1, -.2));

                telemetry.addData("Distance to turn: ", Math.abs(gyro.getIntegratedZValue() - targetHeading));
                telemetry.addData("Target", targetHeading);
                telemetry.addData("Heading", gyro.getIntegratedZValue());
                telemetry.addData("Original Speed", maxSpeed * (Math.abs(gyro.getIntegratedZValue() - targetHeading) / Math.abs(degrees)));
                telemetry.addData("Speed", Range.clip(maxSpeed * (Math.abs(gyro.getIntegratedZValue() - targetHeading) / Math.abs(degrees)), .1, maxSpeed));
                telemetry.update();
                idle();
            }
        } else { //Left
            while (gyro.getIntegratedZValue() < targetHeading && opModeIsActive()) {
                motorLeftFront.setPower(Range.clip(-maxSpeed * (Math.abs(gyro.getIntegratedZValue() - targetHeading) / Math.abs(degrees)), -maxSpeed, -.1));
                motorLeftBack.setPower(Range.clip(-maxSpeed * (Math.abs(gyro.getIntegratedZValue() - targetHeading) / Math.abs(degrees)), -maxSpeed, -.1));
                motorRightFront.setPower(Range.clip(maxSpeed * (Math.abs(gyro.getIntegratedZValue() - targetHeading) / Math.abs(degrees)), .1, maxSpeed));
                motorRightBack.setPower(Range.clip(maxSpeed * (Math.abs(gyro.getIntegratedZValue() - targetHeading) / Math.abs(degrees)), .1, maxSpeed));

                telemetry.addData("Distance to turn: ", Math.abs(gyro.getIntegratedZValue() - targetHeading));
                telemetry.addData("Target", targetHeading);
                telemetry.addData("Heading", gyro.getIntegratedZValue());
                telemetry.addData("Original Speed", maxSpeed * (Math.abs(gyro.getIntegratedZValue() - targetHeading) / Math.abs(degrees)));
                telemetry.addData("Speed", Range.clip(maxSpeed * (Math.abs(gyro.getIntegratedZValue() - targetHeading) / Math.abs(degrees)), .1, maxSpeed));
                telemetry.update();
                idle();
            }
        }

        motorLeftFront.setPower(0);
        motorLeftBack.setPower(0);
        motorRightFront.setPower(0);
        motorRightBack.setPower(0);
        sleep(200);

        telemetry.addData("Distance to turn", Math.abs(gyro.getIntegratedZValue() - targetHeading));
        telemetry.addData("Direction", -1 * (int) Math.signum(degrees));
        telemetry.update();

        if (Math.abs(gyro.getIntegratedZValue() - targetHeading) > 0 && count < 1) {
            //Recurse to correct turn
            turn(Math.abs(gyro.getIntegratedZValue() - targetHeading), direction.equals(OpModeBase.Direction.RIGHT) ? OpModeBase.Direction.LEFT : OpModeBase.Direction.RIGHT, .08, ++count);
        }
    }

    void move(double distance, double maxSpeed) {
        double distanceSign = Math.signum(distance);
        distance = distanceSign * (48.642 * Math.abs(distance) - 267.32);
        int initialHeading = gyro.getIntegratedZValue();

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
            double error = initialHeading - gyro.getIntegratedZValue();
            double targetError = error * kP * distanceSign;

            motorLeftFront.setPower(Range.clip(Range.clip(maxSpeed * (Math.abs(motorLeftFront.getCurrentPosition() - motorLeftFront.getTargetPosition()) / distance), slowdownMin, maxSpeed) - targetError, 0, 1));
            motorLeftBack.setPower(Range.clip(Range.clip(maxSpeed * (Math.abs(motorLeftBack.getCurrentPosition() - motorLeftBack.getTargetPosition()) / distance), slowdownMin, maxSpeed) - targetError, 0, 1));
            motorRightFront.setPower(Range.clip(Range.clip(maxSpeed * (Math.abs(motorRightFront.getCurrentPosition() - motorRightFront.getTargetPosition()) / distance), slowdownMin, maxSpeed) + targetError, 0, 1));
            motorRightBack.setPower(Range.clip(Range.clip(maxSpeed * (Math.abs(motorRightBack.getCurrentPosition() - motorRightBack.getTargetPosition()) / distance), slowdownMin, maxSpeed) + targetError, 0, 1));

            telemetry.addData("Left motor power", motorLeftFront.getPower());
            telemetry.addData("Right motor power", motorRightFront.getPower());
            telemetry.addData("Current Heading", gyro.getIntegratedZValue());
            telemetry.addData("Error", targetError);

            telemetry.addData("Left front position", motorLeftFront.getCurrentPosition());
            telemetry.addData("Left back position", motorLeftBack.getCurrentPosition());
            telemetry.addData("Right front position", motorRightFront.getCurrentPosition());
            telemetry.addData("Right back position", motorRightBack.getCurrentPosition());
            telemetry.update();
            idle();
        }

        motorLeftFront.setPower(0);
        motorLeftBack.setPower(0);
        motorRightFront.setPower(0);
        motorRightBack.setPower(0);
        sleep(300);

        //Correct if robot turned during movement
        if (Math.abs(gyro.getIntegratedZValue() - initialHeading) > 0)
            turn(Math.abs(gyro.getIntegratedZValue() - initialHeading), gyro.getIntegratedZValue() > initialHeading ? OpModeBase.Direction.RIGHT : OpModeBase.Direction.LEFT, .1);
    }

    void driveToWhiteLine(double leftPower, double rightPower) {
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (ods.getRawLightDetected() < odsEdge && opModeIsActive()) { //ODS averages values
            motorLeftFront.setPower(leftPower); //No PID, rides along side of wall
            motorLeftBack.setPower(leftPower);
            motorRightFront.setPower(rightPower);
            motorRightBack.setPower(rightPower);

            telemetry.addData("Left Motor Power", motorLeftFront.getPower());
            telemetry.addData("Right Motor Power", motorRightFront.getPower());
            telemetry.addData("ODS Reading", ods.getRawLightDetected());
            telemetry.update();
            idle();
        }

        motorLeftFront.setPower(0);
        motorLeftBack.setPower(0);
        motorRightFront.setPower(0);
        motorRightBack.setPower(0);
        sleep(300);
    }

    String getColorName() {
        float[] hsv = {0F, 0F, 0F};
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsv);

        if ((hsv[0] < 30 || hsv[0] > 340) && hsv[1] > .2) return "Red";
        else if ((hsv[0] > 170 && hsv[0] < 260) && hsv[1] > .2) return "Blue";
        return "Undefined";
    }

    void shoot() {
        shooter.setPower(1);
        ballStop.setPosition(BALL_STOP_UP);
        sleep(200);
        intake.setPower(1);
        sleep(900);
        shooter.setPower(0);

        //Second ball
        sleep(1800); //Intake still running
        intake.setPower(0);
        shooter.setPower(1);
        sleep(1000);
        shooter.setPower(0);
        sleep(300);
    }

    void moveFast(double distance, double maxSpeed) {
        distance = 48.642 * distance - 267.32;

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
            telemetry.addData("Left motor power", motorLeftFront.getPower());
            telemetry.addData("Right motor power", motorRightFront.getPower());

            telemetry.addData("Left front position", motorLeftFront.getCurrentPosition());
            telemetry.addData("Left back position", motorLeftBack.getCurrentPosition());
            telemetry.addData("Right front position", motorRightFront.getCurrentPosition());
            telemetry.addData("Right back position", motorRightBack.getCurrentPosition());
            telemetry.update();
            idle();
        }

        motorLeftFront.setPower(0);
        motorLeftBack.setPower(0);
        motorRightFront.setPower(0);
        motorRightBack.setPower(0);
        sleep(300);
    }
}