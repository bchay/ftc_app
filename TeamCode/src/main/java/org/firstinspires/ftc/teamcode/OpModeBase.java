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
    //Hardware Declaration

    //Drive Motors
    DcMotor motorLeftFront;
    DcMotor motorLeftBack;
    DcMotor motorRightFront;
    DcMotor motorRightBack;

    //Button Servos
    Servo buttonPresser;

    //Sensor Decleration
    private ModernRoboticsI2cGyro gyro;
    private ColorSensor colorSensor;
    private OpticalDistanceSensor ods;
    ModernRoboticsI2cRangeSensor range;

    //Variables
    final double BUTTON_PRESSER_LEFT = .20;
    final double BUTTON_PRESSER_RIGHT = .89;
    final double BUTTON_PRESSER_NEUTRAL = .55;

    //SharedPreferences Settings Information
    SharedPreferences sharedPreferences;
    Direction moveDirection;

    String allianceColor;
    String location;
    int delay;

    //Autonomous Specific Configuration
    double moveSpeed = .8;
    double turnSpeed = .3;
    private double kP = 0.0160;
    private double slowdownMin = .2;

    enum Direction {
        LEFT, RIGHT;

        private static OpModeBase.Direction[] vals = values();
        public OpModeBase.Direction next() {
            return vals[(this.ordinal() + 1) % vals.length];
        }
    }

    //Initialize all hardware
    public void runOpMode() {
        sharedPreferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);

        //*************** Instantiate hardware devices ***************

        //Drive Motors
        motorLeftFront = hardwareMap.dcMotor.get("left_front");
        motorLeftBack = hardwareMap.dcMotor.get("left_back");
        motorRightFront = hardwareMap.dcMotor.get("right_front");
        motorRightBack = hardwareMap.dcMotor.get("right_back");

        //Button Servos
        buttonPresser = hardwareMap.servo.get("button_presser");

        //Sensor Declaration
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        colorSensor = hardwareMap.colorSensor.get("color");
        ods = hardwareMap.opticalDistanceSensor.get("ods");
        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");

        //*************** Configure hardware devices ***************

        //Configure Motors

        //Drive motors
        motorLeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLeftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Autonomous methods that need RUN_TO_POSITION will set the motors, RUN_USING_ENCODER is required for teleop and gyro turn
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //Configure Servos
        buttonPresser.setPosition(BUTTON_PRESSER_NEUTRAL); //Initially set to left position


        //Configure Sensors

        //Gyro Calibration
        gyro.calibrate();

        //Wait while gyro is calibrating
        while (gyro.isCalibrating() && !isStopRequested()) {
            sleep(50);
            idle();
        }

        //Disable Color Sensor LED
        colorSensor.enableLed(false);

        //Configure SharedPreferences Settings (For Autonomous)
        allianceColor = sharedPreferences.getString("com.qualcomm.ftcrobotcontroller.Autonomous.Color", "null");
        location = sharedPreferences.getString("com.qualcomm.ftcrobotcontroller.Autonomous.Location", "null");
        delay = sharedPreferences.getInt("com.qualcomm.ftcrobotcontroller.Autonomous.Delay", 0);

        if(allianceColor.equals("Blue")) {
            moveDirection = Direction.RIGHT;
        } else {
            moveDirection = Direction.LEFT;
        }

        telemetry.addData("Ready to start program", "");
        telemetry.update();
    }

    void turn(double degrees, OpModeBase.Direction direction, double maxSpeed) { //count is optional, set to 0 if not provided
        if(!opModeIsActive()) return;
        turn(degrees, direction, maxSpeed, 0);
    }

    void turn(double degrees, OpModeBase.Direction direction, double maxSpeed, int count) {
        if(!opModeIsActive()) return;
        if(direction.equals(OpModeBase.Direction.RIGHT)) degrees *= -1; //Negative degree for turning right
        double targetHeading = gyro.getIntegratedZValue() + degrees;

        //Change mode because turn() uses motor power and not motor position
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if(degrees < 0) {
            while(gyro.getIntegratedZValue() > targetHeading && opModeIsActive()) {
                motorLeftFront.setPower(Range.clip(maxSpeed * (Math.abs(gyro.getIntegratedZValue() - targetHeading) / Math.abs(degrees)), .01, maxSpeed));
                motorLeftBack.setPower(Range.clip(maxSpeed * (Math.abs(gyro.getIntegratedZValue() - targetHeading) / Math.abs(degrees)), .01, maxSpeed));
                motorRightFront.setPower(Range.clip(-maxSpeed * (Math.abs(gyro.getIntegratedZValue() - targetHeading) / Math.abs(degrees)), -maxSpeed, -.01));
                motorRightBack.setPower(Range.clip(-maxSpeed * (Math.abs(gyro.getIntegratedZValue() - targetHeading) / Math.abs(degrees)), -maxSpeed, -.01));

                telemetry.addData("Distance to turn: ", Math.abs(gyro.getIntegratedZValue() - targetHeading));
                telemetry.addData("Target", targetHeading);
                telemetry.addData("Heading", gyro.getIntegratedZValue());
                telemetry.addData("Original Speed", maxSpeed * (Math.abs(gyro.getIntegratedZValue() - targetHeading) / Math.abs(degrees)));
                telemetry.addData("Speed", Range.clip(maxSpeed * (Math.abs(gyro.getIntegratedZValue() - targetHeading) / Math.abs(degrees)), .01, maxSpeed));
                telemetry.update();
                idle();
            }
        } else { //Left
            while (gyro.getIntegratedZValue() < targetHeading && opModeIsActive()) {
                motorLeftFront.setPower(Range.clip(-maxSpeed * (Math.abs(gyro.getIntegratedZValue() - targetHeading) / Math.abs(degrees)), -maxSpeed, -.01));
                motorLeftBack.setPower(Range.clip(-maxSpeed * (Math.abs(gyro.getIntegratedZValue() - targetHeading) / Math.abs(degrees)), -maxSpeed, -.01));
                motorRightFront.setPower(Range.clip(maxSpeed * (Math.abs(gyro.getIntegratedZValue() - targetHeading) / Math.abs(degrees)), .01, maxSpeed));
                motorRightBack.setPower(Range.clip(maxSpeed * (Math.abs(gyro.getIntegratedZValue() - targetHeading) / Math.abs(degrees)), .01, maxSpeed));

                telemetry.addData("Distance to turn: ", Math.abs(gyro.getIntegratedZValue() - targetHeading));
                telemetry.addData("Target", targetHeading);
                telemetry.addData("Heading", gyro.getIntegratedZValue());
                telemetry.addData("Original Speed", maxSpeed * (Math.abs(gyro.getIntegratedZValue() - targetHeading) / Math.abs(degrees)));
                telemetry.addData("Speed", Range.clip(maxSpeed * (Math.abs(gyro.getIntegratedZValue() - targetHeading) / Math.abs(degrees)), .01, maxSpeed));
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

        if(Math.abs(gyro.getIntegratedZValue() - targetHeading) > 0 && count < 1) {
            //Recurse to correct turn
            turn(Math.abs(gyro.getIntegratedZValue() - targetHeading), direction.equals(OpModeBase.Direction.RIGHT) ? OpModeBase.Direction.LEFT : OpModeBase.Direction.RIGHT, .08, ++count);
        }
    }

    void move(double distance, double maxSpeed) {
        distance = 47.973 * distance + 475.3;
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

        while(motorLeftFront.isBusy() && motorLeftBack.isBusy() && motorRightFront.isBusy() && motorRightBack.isBusy() && opModeIsActive()) {
            //Only one encoder target must be reached
            double error = initialHeading - gyro.getIntegratedZValue();
            double targetError =  error * kP;
            motorLeftFront.setPower(Range.clip(Range.clip(maxSpeed * (Math.abs(motorLeftFront.getCurrentPosition() - motorLeftFront.getTargetPosition()) / distance), slowdownMin, maxSpeed) - targetError, 0, 1));
            motorLeftBack.setPower(Range.clip(Range.clip(maxSpeed * (Math.abs(motorLeftBack.getCurrentPosition() - motorLeftBack.getTargetPosition()) / distance), slowdownMin, maxSpeed) - targetError, 0, 1));
            motorRightFront.setPower(Range.clip(Range.clip(maxSpeed * (Math.abs(motorRightFront.getCurrentPosition() - motorRightFront.getTargetPosition()) / distance), slowdownMin, maxSpeed) + targetError, 0, 1));
            motorRightBack.setPower(Range.clip(Range.clip(maxSpeed * (Math.abs(motorRightBack.getCurrentPosition() - motorRightBack.getTargetPosition()) / distance), slowdownMin, maxSpeed) + targetError, 0, 1));

            telemetry.addData("Left motor power", motorLeftFront.getPower());
            telemetry.addData("Right motor power", motorRightFront.getPower());
            telemetry.addData("Initial Heading", initialHeading);
            telemetry.addData("Current Heading", gyro.getIntegratedZValue());
            telemetry.addData("Error", targetError);
            telemetry.update();
            idle();
        }

        motorLeftFront.setPower(0);
        motorLeftBack.setPower(0);
        motorRightFront.setPower(0);
        motorRightBack.setPower(0);
        sleep(200);

        //Correct if robot turned during movement
        if(Math.abs(gyro.getIntegratedZValue() - initialHeading) > 0) turn(Math.abs(gyro.getIntegratedZValue() - initialHeading), gyro.getIntegratedZValue() > initialHeading ? OpModeBase.Direction.RIGHT : OpModeBase.Direction.LEFT, .1);
    }

    void driveToWhiteLine(double power) {
        //Uses encoders for PID, no target for RUN_TO_POSITION
        int initialHeading = gyro.getIntegratedZValue();
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (ods.getLightDetected() < 5 && opModeIsActive()) { //ODS averages values
            double error = initialHeading - gyro.getIntegratedZValue();
            double targetError =  error * .015; //Value must be adjusted

            motorLeftFront.setPower(Range.clip(power - targetError, 0, 1));
            motorLeftBack.setPower(Range.clip(power - targetError, 0, 1));
            motorRightFront.setPower(Range.clip(power + targetError, 0, 1));
            motorRightBack.setPower(Range.clip(power + targetError, 0, 1));

            telemetry.addData("Left Motor Power", motorLeftFront.getPower());
            telemetry.addData("Right Motor Power", motorRightFront.getPower());
            telemetry.addData("ODS Reading", ods.getRawLightDetected());
            telemetry.addData("Gyro heading", gyro.getIntegratedZValue());
            telemetry.update();
            idle();
        }

        motorLeftFront.setPower(0);
        motorLeftBack.setPower(0);
        motorRightFront.setPower(0);
        motorRightBack.setPower(0);
        sleep(400);

        int finalHeading = gyro.getIntegratedZValue();
        if (Math.abs(initialHeading - finalHeading) > 0) {
            turn(Math.abs(initialHeading - finalHeading), initialHeading > finalHeading ? OpModeBase.Direction.LEFT : OpModeBase.Direction.RIGHT, .1); //Negative degrees for right
        }
    }

    void moveUntilDistance(double distance, double speed) {
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (distance < range.getDistance(DistanceUnit.INCH)) { //Move forward
            while (range.getDistance(DistanceUnit.INCH) > distance && opModeIsActive()) {
                motorLeftFront.setPower(speed);
                motorLeftBack.setPower(speed);
                motorRightFront.setPower(speed);
                motorRightBack.setPower(speed);

                telemetry.addData("Left Motor Power", motorLeftFront.getPower());
                telemetry.addData("Right Motor Power", motorRightFront.getPower());
                telemetry.addData("Range", range.getDistance(DistanceUnit.INCH));
                sleep(50);
                idle();
            }
        } else {
            while (range.getDistance(DistanceUnit.INCH) > distance && opModeIsActive()) {
                motorLeftFront.setPower(-speed);
                motorLeftBack.setPower(-speed);
                motorRightFront.setPower(-speed);
                motorRightBack.setPower(-speed);

                telemetry.addData("Left Motor Power", motorLeftFront.getPower());
                telemetry.addData("Right Motor Power", motorRightFront.getPower());
                telemetry.addData("Range", range.getDistance(DistanceUnit.INCH));
                sleep(50);
                idle();
            }
        }
            motorLeftFront.setPower(0);
            motorLeftBack.setPower(0);
            motorRightFront.setPower(0);
            motorRightBack.setPower(0);

            sleep(300);
    }

    void followLine(double speed) {
        double initialHeading = gyro.getIntegratedZValue();
        while(range.getDistance(DistanceUnit.INCH) < 10) {
            if(ods.getLightDetected() < 5) {
                motorLeftFront.setPower(speed);
                motorLeftBack.setPower(speed);
                motorRightFront.setPower(speed);
                motorRightBack.setPower(speed);
            } else if (gyro.getIntegratedZValue() - initialHeading > 0) { //Robot has moved right
                motorLeftFront.setPower(speed);
                motorLeftBack.setPower(speed);
                motorRightFront.setPower(speed/2);
                motorRightBack.setPower(speed/2);
            } else {
                motorLeftFront.setPower(speed/2);
                motorLeftBack.setPower(speed/2);
                motorRightFront.setPower(speed);
                motorRightBack.setPower(speed);
            }
        }
    }

    String getColorName() {
        float[] hsv = {0F, 0F, 0F};
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsv);

        if ((hsv[0] < 30 || hsv[0] > 340) && hsv[1] > .2) return "Red";
        else if ((hsv[0] > 170 && hsv[0] < 260) && hsv[1] > .2) return "Blue";
        return "Undefined";
    }
}

