package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.graphics.Color;
import android.preference.PreferenceManager;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "Autonomous", group = "Main Code")
public class AutonomousCode extends LinearOpMode {

    //Hardware Declaration
    private DcMotor motorLeft; //Controls both left side motors
    private DcMotor motorRight; //Controls both right side motors

    private DcMotor ballShooter;
    private DcMotor ballLift;

    private Servo buttonPresserLeft;
    private Servo buttonPresserRight;

    private Servo ballKicker;
    private Servo ballStopper;

    private ModernRoboticsI2cGyro gyro;
    private ColorSensor colorSensor;
    private OpticalDistanceSensor ods;

    //Variables
    private final double ENCODER_RATIO = 1;
    //58.25 in / 8000 ticks

    private final double BUTTON_PRESSER_LEFT_IN = .3;
    private final double BUTTON_PRESSER_RIGHT_IN = .1;

    private final double BUTTON_PRESSER_LEFT_OUT = 1;
    private final double BUTTON_PRESSER_RIGHT_OUT = .7;

    //Starting positions of ball manipulation servos
    private final double BALL_KICKER_DOWN = .65;
    private final double BALL_STOPPER_OUT = .5;

    private final double BALL_KICKER_UP = .3;
    private final double BALL_STOPPER_IN = .25;

    private double moveSpeed = .9;
    private double turnSpeed = .45;

    private float[] hsv = {0F, 0F, 0F};

    SharedPreferences sharedPreferences;

    private enum Direction {
        LEFT, RIGHT;

        private static Direction[] vals = values();
        public Direction next() {
            return vals[(this.ordinal() + 1) % vals.length];
        }
    }


    Direction moveDirection;

    public void runOpMode() {

        sharedPreferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);

        //Hardware Instantiation
        motorLeft = hardwareMap.dcMotor.get("left");
        motorRight = hardwareMap.dcMotor.get("right");

        ballShooter = hardwareMap.dcMotor.get("ball_shooter");
        ballShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        ballLift = hardwareMap.dcMotor.get("ball_lift");
        ballLift.setDirection(DcMotorSimple.Direction.REVERSE);

        buttonPresserLeft = hardwareMap.servo.get("button_left");
        buttonPresserRight = hardwareMap.servo.get("button_right");

        ballKicker = hardwareMap.servo.get("ball_kicker");
        ballStopper = hardwareMap.servo.get("ball_stopper");

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        colorSensor = hardwareMap.colorSensor.get("color_front");
        ods = hardwareMap.opticalDistanceSensor.get("ods");

        //Position Servos
        buttonPresserLeft.setPosition(BUTTON_PRESSER_LEFT_IN);
        buttonPresserRight.setPosition(BUTTON_PRESSER_RIGHT_IN);

        ballKicker.setPosition(BALL_KICKER_DOWN);
        ballStopper.setPosition(BALL_STOPPER_OUT);

        //Motor Direction
        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        //Motor RunMode
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ballShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ballLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ballShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ballLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Encoders are reset", "");
        telemetry.update();

        //Gyro Calibration
        gyro.calibrate();

        //Wait while gyro is calibrating
        while (gyro.isCalibrating() && !isStopRequested()) {
            telemetry.addData("Gyroscope is currently calibrating.", "");
            telemetry.update();
            idle();
            sleep(50);
        }

        telemetry.addData("Gyroscope is calibrated.", "");
        telemetry.update();

        //Disable Color Sensor LED
        colorSensor.enableLed(false);

        String allianceColor = sharedPreferences.getString("com.qualcomm.ftcrobotcontroller.Autonomous.Color", "null");
        String location = sharedPreferences.getString("com.qualcomm.ftcrobotcontroller.Autonomous.Location", "null");
        int delay = sharedPreferences.getInt("com.qualcomm.ftcrobotcontroller.Autonomous.Delay", 0);

        telemetry.addData("Ready to Start Program", "");
        telemetry.update();

        waitForStart();

        sleep(delay);

        //Beginning of Actual Code

        if(allianceColor.equals("Blue")) {
            moveDirection = Direction.RIGHT;
        } else {
            moveDirection = Direction.LEFT;
        }

        //Robot begins third tile away from corner vortex wall, wheels touching next full tile next to vortex
        if(location.equals("Close")) {
            move(12, moveSpeed);
            turn(48, moveDirection, turnSpeed);
            move(25, 1); //Approach white line
            driveToWhiteLine(.4);
            move(3.5, .5); //Position robot so that servos are in correct location
            turn(42, moveDirection, turnSpeed);
            move(3.5, moveSpeed);

            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsv);
            String beaconColor = getColorName(hsv);

            if(beaconColor.equals(allianceColor)) {
                buttonPresserRight.setPosition(BUTTON_PRESSER_RIGHT_OUT);
            } else {
                buttonPresserLeft.setPosition(BUTTON_PRESSER_LEFT_OUT);
            }

            sleep(400);
            move(15, .65); //Push beacon

            buttonPresserRight.setPosition(BUTTON_PRESSER_RIGHT_IN);
            buttonPresserLeft.setPosition(BUTTON_PRESSER_LEFT_IN);

            move(-5, moveSpeed); //Back away from wall
            turn(90, moveDirection.next(), turnSpeed);
            move(40, moveSpeed); //Drive to second beacon
            driveToWhiteLine(.4);
            move(4, .5);
            turn(90, moveDirection, turnSpeed);

            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsv);
            beaconColor = getColorName(hsv);

            if(beaconColor.equals(allianceColor)) {
                buttonPresserRight.setPosition(BUTTON_PRESSER_RIGHT_OUT);
            } else {
                buttonPresserLeft.setPosition(BUTTON_PRESSER_LEFT_OUT);
            }

            sleep(400);
            move(16, .65); //Push beacon
        }
    }

    private void turn(int degrees, Direction direction, double maxSpeed) { //count is optional, set to 0 if not provided
        turn(degrees, direction, maxSpeed, 0);
    }

    private void turn(int degrees, Direction direction, double maxSpeed, int count) {
        if(!opModeIsActive()) return;
        if(direction.equals(Direction.RIGHT)) degrees *= -1; //Negative degree for turning right
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
            turn(Math.abs(gyro.getIntegratedZValue() - targetHeading), direction.equals(Direction.RIGHT) ? Direction.LEFT : Direction.RIGHT, .1, ++count);
        }
    }

    private void move(double distance, double maxSpeed) {
        //distance *= ENCODER_RATIO;
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

            if(Math.abs(gyro.getIntegratedZValue() - initialHeading) > 0) {
                if(gyro.getIntegratedZValue() > initialHeading) { //Robot has turned to the left
                    motorLeft.setPower(Range.clip(maxSpeed + .4, 0, 1));
                    motorRight.setPower(Range.clip(maxSpeed - .4, 0, 1));
                } else { //Robot has turned to the right
                    motorLeft.setPower(Range.clip(maxSpeed - .4, 0, 1));
                    motorRight.setPower(Range.clip(maxSpeed + .4, 0, 1));
                }
            } else {
                motorLeft.setPower(maxSpeed);
                motorRight.setPower(maxSpeed);
            }

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
        if(Math.abs(gyro.getIntegratedZValue() - initialHeading) > 0) turn(Math.abs(gyro.getIntegratedZValue() - initialHeading), gyro.getIntegratedZValue() > initialHeading ? Direction.RIGHT : Direction.LEFT, .2);
    }

    private void driveToWhiteLine(double power) {
        //Uses encoders for PID, no target for RUN_TO_POSITION
        int initialHeading = gyro.getIntegratedZValue(); //0
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(ods.getRawLightDetected() < .02 && opModeIsActive()) { //.03 is white line //.8-.9 is white, ODS averages values it sees
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
        if(Math.abs(initialHeading - finalHeading) > 0) {
            turn(Math.abs(initialHeading - finalHeading), initialHeading > finalHeading ? Direction.LEFT : Direction.RIGHT, turnSpeed); //Negative degrees for right
        }
    }

    private String getColorName(float[] hsv) {
        if ((hsv[0] < 30 || hsv[0] > 340) && hsv[1] > .2) return "Red";
        else if ((hsv[0] > 170 && hsv[0] < 260) && hsv[1] > .2) return "Blue";
        return "Undefined";
    }

    private void shoot() {
        ballShooter.setPower(.23);
        ballLift.setPower(.2);
        sleep(4000);
        ballStopper.setPosition(BALL_STOPPER_IN);

        //Launch first ball
        sleep(300);
        ballKicker.setPosition(BALL_KICKER_UP);
        ballLift.setPower(1);
        sleep(350);
        ballStopper.setPosition(BALL_STOPPER_OUT);
        sleep(1150);
        ballStopper.setPosition(BALL_STOPPER_IN);

        sleep(300);

        //Launch second ball
        ballKicker.setPosition(BALL_KICKER_DOWN);
        sleep(500);
        ballKicker.setPosition(BALL_KICKER_UP);
        sleep(1500);

        //Reset
        ballLift.setPower(0);
        ballShooter.setPower(0);
        ballStopper.setPosition(BALL_STOPPER_OUT);
        ballKicker.setPosition(BALL_KICKER_DOWN);
    }
}

//1000 / 14.8 = 67.5675675676 ticks / in
//2000 / 25.2 = 79.3650793651
//3000 / 36.4 = 82.4175824176
//4000 / 47.5 = 84.2105263158
//5000 / 57.4 = 87.1080139373
//6000 / 69.35 = 86.5176640231
//6500 / 76.1 = 85.4139290407
//7000 / 81 = 86.4197530864
//7500 / 87 = 86.2068965517
//8000 / 93.5 = 85.5614973262
//9000 / 103.8 = 86.7052023121
//10000 / 114.5 = 87.3362445415