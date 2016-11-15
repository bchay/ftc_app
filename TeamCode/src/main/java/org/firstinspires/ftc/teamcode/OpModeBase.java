package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public abstract class OpModeBase extends LinearOpMode {
    //Hardware Declaration

    //Drive Motors
    DcMotor motorLeft; //Left motors are electrically linked
    DcMotor motorRight; //Right motors are electrically linked

    //Other Motors
    DcMotor ballShooter;
    DcMotor ballLift;

    //Button Servos
    Servo buttonPresserLeft;
    Servo buttonPresserRight;

    //Servos for shooting
    Servo ballKicker;
    Servo ballStopper;

    //Sensor Decleration
    ModernRoboticsI2cGyro gyro;
    ColorSensor colorSensor;
    OpticalDistanceSensor ods;

    //Variables
    final double BUTTON_PRESSER_LEFT_IN = .3; //Default state
    final double BUTTON_PRESSER_LEFT_OUT = 1;

    final double BUTTON_PRESSER_RIGHT_IN = .1; //Default state
    final double BUTTON_PRESSER_RIGHT_OUT = .7;

    //Starting positions of ball manipulation servos
    final double BALL_KICKER_DOWN = .65; //Default state
    final double BALL_KICKER_UP = .3;

    final double BALL_STOPPER_OUT = .5; //Default state
    final double BALL_STOPPER_IN = .25;

    //SharedPreferences Settings Information
    SharedPreferences sharedPreferences;
    Direction moveDirection;

    String allianceColor;
    String location;
    int delay;

    //Autonomous Specific Configuration
    double moveSpeed = .9;
    double turnSpeed = .45;

    float[] hsv = {0F, 0F, 0F}; //Used to store color sensor readings

    enum Direction {
        LEFT, RIGHT;

        private static OpModeBase.Direction[] vals = values();
        public OpModeBase.Direction next() {
            return vals[(this.ordinal() + 1) % vals.length];
        }
    }


    //Teleop Configuration
    boolean ballShooterOn = true;
    boolean ballShooterTriggered = false;

    //Teleop Motor Configuration
    double shooterPower = .23;

    HardwareMap hardware = null;

    //Initialize all hardware
    public void init(HardwareMap hardware) {
        this.hardware = hardware;
        sharedPreferences = PreferenceManager.getDefaultSharedPreferences(hardware.appContext);

        //*************** Instantiate hardware devices ***************

        //Drive Motors
        motorLeft = hardware.dcMotor.get("left");
        motorRight = hardware.dcMotor.get("right");

        //Other Motors
        ballShooter = hardware.dcMotor.get("ball_shooter");
        ballLift = hardware.dcMotor.get("ball_lift");

        //Button Servos
        buttonPresserLeft = hardware.servo.get("button_left");
        buttonPresserRight = hardware.servo.get("button_right");

        //Servos for shooting
        ballKicker = hardware.servo.get("ball_kicker");
        ballStopper = hardware.servo.get("ball_stopper");

        //Sensor Declaration
        gyro = (ModernRoboticsI2cGyro) hardware.gyroSensor.get("gyro");
        colorSensor = hardware.colorSensor.get("color_front");
        ods = hardware.opticalDistanceSensor.get("ods");

        //*************** Configure hardware devices ***************

        //Configure Motors

        //Drive motors
        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Autonomous methods that need RUN_TO_POSITION will set the motors, RUN_USING_ENCODER is required for teleop
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Shooter Motors
        ballShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        ballLift.setDirection(DcMotorSimple.Direction.REVERSE);

        ballShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ballLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ballShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ballLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //Configure Servos
        buttonPresserLeft.setPosition(BUTTON_PRESSER_LEFT_IN);
        buttonPresserRight.setPosition(BUTTON_PRESSER_RIGHT_IN);

        ballKicker.setPosition(BALL_KICKER_DOWN);
        ballStopper.setPosition(BALL_STOPPER_OUT);

        //Configure Sensors

        //Gyro Calibration
        gyro.calibrate();

        //Wait while gyro is calibrating
        while (gyro.isCalibrating()) {
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

    void turn(int degrees, OpModeBase.Direction direction, double maxSpeed) { //count is optional, set to 0 if not provided
        turn(degrees, direction, maxSpeed, 0);
    }

    void turn(int degrees, OpModeBase.Direction direction, double maxSpeed, int count) {
        if(!opModeIsActive()) return;
        if(direction.equals(OpModeBase.Direction.RIGHT)) degrees *= -1; //Negative degree for turning right
        int targetHeading = gyro.getIntegratedZValue() + degrees;

        //Change mode because turn() uses motor power and not motor position
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if(degrees < 0) {
            while(gyro.getIntegratedZValue() > targetHeading && opModeIsActive()) {
                motorLeft.setPower(Math.abs(gyro.getIntegratedZValue() - targetHeading) > 30 ? maxSpeed: .1);
                motorRight.setPower(Math.abs(gyro.getIntegratedZValue() - targetHeading) > 30 ? -maxSpeed: -.1);

                telemetry.addData("Left power", motorLeft.getPower());
                telemetry.addData("Right power", motorRight.getPower());

                telemetry.addData("Target", targetHeading);

                telemetry.addData("Distance to turn: ", Math.abs(gyro.getIntegratedZValue() - targetHeading));
                telemetry.addData("Heading", gyro.getIntegratedZValue());
                telemetry.update();
                idle();
            }
        } else { //Left
            while (gyro.getIntegratedZValue() < targetHeading && opModeIsActive()) {
                motorLeft.setPower(Math.abs(gyro.getIntegratedZValue() - targetHeading) > 25 ? -maxSpeed: -.15);
                motorRight.setPower(Math.abs(gyro.getIntegratedZValue() - targetHeading) > 25 ? maxSpeed: .15);

                telemetry.addData("Left power", motorLeft.getPower());
                telemetry.addData("Right power", motorRight.getPower());

                telemetry.addData("Target", targetHeading);

                telemetry.addData("Distance to turn: ", Math.abs(gyro.getIntegratedZValue() - targetHeading));
                telemetry.addData("Heading", gyro.getIntegratedZValue());
                telemetry.update();
                idle();
            }
        }

        motorLeft.setPower(0);
        motorRight.setPower(0);
        sleep(200);

        telemetry.addData("Distance to turn", Math.abs(gyro.getIntegratedZValue() - targetHeading));
        telemetry.addData("Direction", -1 * (int) Math.signum(degrees));
        telemetry.update();

        if(Math.abs(gyro.getIntegratedZValue() - targetHeading) > 0 && count < 1) {
            //Recurse to correct turn
            turn(Math.abs(gyro.getIntegratedZValue() - targetHeading), direction.equals(OpModeBase.Direction.RIGHT) ? OpModeBase.Direction.LEFT : OpModeBase.Direction.RIGHT, .1, ++count);
        }
    }

    void move(double distance, double maxSpeed) {
        distance  = 89.34520401 * distance - 253.625438;
        int initialHeading = gyro.getIntegratedZValue();

        //Change mode because move() uses setTargetPosition()
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLeft.setTargetPosition((int) (motorLeft.getCurrentPosition() + distance));
        motorRight.setTargetPosition((int) (motorRight.getCurrentPosition() + distance));

        motorLeft.setPower(maxSpeed);
        motorRight.setPower(maxSpeed);

        while(motorLeft.isBusy() && motorRight.isBusy() && opModeIsActive()) {
            //Only one encoder target must be reached
            double targetError = (initialHeading - gyro.getHeading()) * .5; //Value must be tested
            motorLeft.setPower(Range.clip(maxSpeed + targetError, 0, 1));
            motorRight.setPower(Range.clip(maxSpeed - targetError, 0, 1));

            telemetry.addData("Heading", gyro.getIntegratedZValue());
            telemetry.addData("Left motor power", motorLeft.getPower());
            telemetry.addData("Right motor power", motorRight.getPower());

            telemetry.update();
            idle();
        }

        motorLeft.setPower(0);
        motorRight.setPower(0);
        sleep(350);

        //Correct if robot turned during movement
        if(Math.abs(gyro.getIntegratedZValue() - initialHeading) > 0) turn(Math.abs(gyro.getIntegratedZValue() - initialHeading), gyro.getIntegratedZValue() > initialHeading ? OpModeBase.Direction.RIGHT : OpModeBase.Direction.LEFT, .2);
    }

    void driveToWhiteLine(double power) {
        //Uses encoders for PID, no target for RUN_TO_POSITION
        int initialHeading = gyro.getIntegratedZValue(); //0
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (ods.getRawLightDetected() < .02 && opModeIsActive()) { //.03 is white line //.8-.9 is white, ODS averages values it sees
            motorLeft.setPower(power); //Adjust speed if robot is turned
            motorRight.setPower(power);

            telemetry.addData("ODS Reading", ods.getRawLightDetected());
            telemetry.addData("Gyro heading", gyro.getIntegratedZValue());
            telemetry.update();
            idle();
        }

        motorLeft.setPower(0);
        motorRight.setPower(0);
        sleep(400);

        int finalHeading = gyro.getIntegratedZValue();
        if (Math.abs(initialHeading - finalHeading) > 0) {
            turn(Math.abs(initialHeading - finalHeading), initialHeading > finalHeading ? OpModeBase.Direction.LEFT : OpModeBase.Direction.RIGHT, turnSpeed); //Negative degrees for right
        }
    }

    void shoot() {
        ballShooter.setPower(.33);
        ballLift.setPower(.2);
        sleep(5000);
        ballStopper.setPosition(BALL_STOPPER_IN);

        //Launch first ball
        sleep(500);
        ballKicker.setPosition(BALL_KICKER_UP); //Move kicker to hit ball
        ballLift.setPower(1);
        sleep(1200);

        //Reset
        ballLift.setPower(0);
        ballShooter.setPower(0);
        ballStopper.setPosition(BALL_STOPPER_OUT);
        ballKicker.setPosition(BALL_KICKER_DOWN);
    }

    String getColorName(float[] hsv) {
        if ((hsv[0] < 30 || hsv[0] > 340) && hsv[1] > .2) return "Red";
        else if ((hsv[0] > 170 && hsv[0] < 260) && hsv[1] > .2) return "Blue";
        return "Undefined";
    }
}

