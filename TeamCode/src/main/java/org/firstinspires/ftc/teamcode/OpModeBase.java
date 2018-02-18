package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

/**
 * Contains variables and methods used to control the robot. Autonomous and TeleOp classes are subclasses of OpModeBase.
 */

abstract public class OpModeBase extends LinearOpMode {
    //*************** Declare Hardware Devices ***************

    //Motors
    DcMotor motorLeftFront;
    DcMotor motorLeftBack;
    DcMotor motorRightFront;
    DcMotor motorRightBack;

    DcMotor glyphLift;
    DcMotor leftIntake;
    DcMotor rightIntake;
    DcMotor moveIntake;

    //Servos
    Servo colorSensorArm; //Arm is on right side of robot looking at robot from back
    Servo colorSensorRotator;

    Servo glyphFlipper;
    Servo glyphStopper;
    Servo glyphLever;

    //Sensors
    private BNO055IMU imu;
    private ColorSensor colorSensor; //Color sensor is pointing towards right jewel

    //General Constants
    private static final double COLOR_SENSOR_ARM_INITIAL_AUTONOMOUS = 1;
    static final double COLOR_ROTATOR_INITIAL_AUTONOMOUS = .453;
    private static final double COLOR_ROTATOR_INITIAL_TELEOP = .646;
    private static final double COLOR_SENSOR_ARM_INITIAL_TELEOP = .646;

    static final double GLYPH_FLIPPER_FLAT = .29;
    static final double GLYPH_FLIPPER_PARTIALLY_UP = .416;
    static final double GLYPH_FLIPPER_VERTICAL = .73;

    static final double GLYPH_STOPPER_DOWN = .365;
    static final double GLYPH_STOPPER_UP = .584;

    static final double GLYPH_LEVER_DOWN_FLIPPER = 1;
    static final double GLYPH_LEVER_UP = .608;
    static final double GLYPH_LEVER_DOWN_INTAKE = .075;

    //Autonomous Specific Configuration
    private double moveSpeedMin = .2;
    double moveSpeedMax = .95;
    private double ticksRatioForward = 5000 / 56; //Ticks / inch
    private double ticksRatioStrafe = 5000 / 45;

    double turnSpeed = .5; //Speed is ramped down as turn proceeds

    enum Direction {
        FORWARD, BACKWARD, LEFT, RIGHT;

        //Taken from http://stackoverflow.com/a/17006263
        private static OpModeBase.Direction[] vals = values();

        public OpModeBase.Direction next() {
            return vals[(this.ordinal() + 1) % vals.length];
        }
    }

    public enum OpModeType {
        AUTONOMOUS, TELEOP;
    }

    //Autonomous instance variables
    VuMarkReader vuMarkReader;
    ElapsedTime time; //Used for sensor reading timeout

    //General Variables
    private double previousHeading = 0;
    private double integratedHeading = 0;

    /**
     * Configures all parts of the robot.
     */
    public void runOpMode(OpModeType opModeType) {
        mapHardware();

        //Drive motors

        //Motors are flipped for teleop
        if (opModeType.equals(OpModeType.AUTONOMOUS)) {
            motorLeftFront.setDirection(FORWARD);
            motorLeftBack.setDirection(FORWARD);
            motorRightFront.setDirection(REVERSE);
            motorRightBack.setDirection(REVERSE);
        } else {
            motorLeftFront.setDirection(REVERSE);
            motorLeftBack.setDirection(REVERSE);
            motorRightFront.setDirection(FORWARD);
            motorRightBack.setDirection(FORWARD);
        }

        motorLeftFront.setMode(STOP_AND_RESET_ENCODER);
        motorLeftBack.setMode(STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(STOP_AND_RESET_ENCODER);
        motorRightBack.setMode(STOP_AND_RESET_ENCODER);

        motorLeftFront.setMode(RUN_USING_ENCODER); //Autonomous methods that need a different mode will set it, RUN_WITHOUT_ENCODER used for teleop
        motorLeftBack.setMode(RUN_USING_ENCODER);
        motorRightFront.setMode(RUN_USING_ENCODER);
        motorRightBack.setMode(RUN_USING_ENCODER);

        motorLeftFront.setZeroPowerBehavior(BRAKE); //Default is FLOAT
        motorLeftBack.setZeroPowerBehavior(BRAKE);
        motorRightFront.setZeroPowerBehavior(BRAKE);
        motorRightBack.setZeroPowerBehavior(BRAKE);

        glyphLift.setMode(STOP_AND_RESET_ENCODER);
        glyphLift.setMode(RUN_USING_ENCODER);
        glyphLift.setZeroPowerBehavior(BRAKE);

        leftIntake.setZeroPowerBehavior(BRAKE);
        rightIntake.setZeroPowerBehavior(BRAKE);

        //Servos initialized only during autonomous init period
        //Initialize autonomous specific variables
        if (opModeType.equals(OpModeType.AUTONOMOUS)) {
            initializeServos(OpModeType.AUTONOMOUS);
            initializeIMU();

            vuMarkReader = new VuMarkReader(hardwareMap);
            time = new ElapsedTime();
            displayInitialTelemetry();
        }
    }

    private void mapHardware() {
        //Drive Motors
        motorLeftFront = hardwareMap.dcMotor.get("left front");
        motorLeftBack = hardwareMap.dcMotor.get("left back");
        motorRightFront = hardwareMap.dcMotor.get("right front");
        motorRightBack = hardwareMap.dcMotor.get("right back");

        leftIntake = hardwareMap.dcMotor.get("left intake");
        rightIntake = hardwareMap.dcMotor.get("right intake");
        moveIntake = hardwareMap.dcMotor.get("move intake");

        glyphLift = hardwareMap.dcMotor.get("glyph lift");

        //Sensors
        imu = hardwareMap.get(BNO055IMU.class, "imu 1");
        colorSensor = hardwareMap.colorSensor.get("color");

        //Servos
        colorSensorArm = hardwareMap.servo.get("color arm");
        colorSensorRotator = hardwareMap.servo.get("color rotator");

        glyphFlipper = hardwareMap.servo.get("glyph flipper");
        glyphStopper = hardwareMap.servo.get("glyph stopper");
        glyphLever = hardwareMap.servo.get("glyph lever");
    }

    void initializeServos(OpModeType opModeType) {
        if(opModeType.equals(OpModeType.TELEOP)) {
            colorSensorArm.setPosition(COLOR_SENSOR_ARM_INITIAL_TELEOP);
            colorSensorRotator.setPosition(COLOR_ROTATOR_INITIAL_TELEOP);
        } else {
            colorSensorArm.setPosition(COLOR_SENSOR_ARM_INITIAL_AUTONOMOUS);
            colorSensorRotator.setPosition(COLOR_ROTATOR_INITIAL_AUTONOMOUS);
        }

        glyphFlipper.setPosition(GLYPH_FLIPPER_FLAT);
        glyphStopper.setPosition(GLYPH_STOPPER_DOWN);
        glyphLever.setPosition(GLYPH_LEVER_DOWN_INTAKE);
    }

    private void initializeIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }


    //*************** Autonomous Methods ***************

    private void displayInitialTelemetry() {
        while(!isStarted() && !isStopRequested()) { //Display distance telemetry for robot alignment
            telemetry.addData("Ready to start the program.", "");
            telemetry.addData("Encoders", motorLeftFront.getCurrentPosition() + motorLeftBack.getCurrentPosition() + motorRightFront.getCurrentPosition() + motorRightBack.getCurrentPosition());
            telemetry.addData("Heading", getIntegratedHeading());
            telemetry.addData("VuMark", vuMarkReader.getVuMark());
            telemetry.update();
        }
    }

    void hitJewel(String allianceColor) {
        colorSensorArm.setPosition(.198); //Move forward
        sleep(1000);
        colorSensorRotator.setPosition(.72); //Center arm between jewels
        sleep(500);
        colorSensorArm.setPosition(.141); //Move down next to right jewel
        sleep(500);

        //Knock off jewel of opposing alliance color
        time.reset();
        while(getColor().equals("Unknown") && opModeIsActive() && time.milliseconds() < 1000) { //Timeout at 1 second
            telemetry.addData("Color", "Unknown");
            telemetry.update();
        }

        //Color sensor reads left jewel
        telemetry.addData("Color", getColor());
        telemetry.update();

        if(getColor().equals(allianceColor) && !getColor().equals("Unknown")) {
            colorSensorRotator.setPosition(1); //Move to hit left jewel
            sleep(500);
        } else if(!getColor().equals("Unknown")) { //Color is still detected, is opposing alliance's color
            colorSensorRotator.setPosition(.376); //Move to hit right jewel
            sleep(500);
        } else { //Color not detected, move arm up and right
            colorSensorArm.setPosition(.25);
            sleep(500);
            colorSensorRotator.setPosition(.806);
            sleep(500);
        }

        //Return jewel arm to upright position so that it does not get in the way of the remainder of the autonomous
        colorSensorArm.setPosition(.646); //Move arm up
        sleep(1000);
        colorSensorRotator.setPosition(.329); //Move rotator behind metal piece to stop it from falling after teleop ends
        sleep(500);
    }

    RelicRecoveryVuMark readVuMark(String allianceColor) {
        //Read VuMark to determine cryptobox key
        RelicRecoveryVuMark vuMark = vuMarkReader.getVuMark();

        time.reset();
        while(vuMark.equals(RelicRecoveryVuMark.UNKNOWN) && opModeIsActive() && time.milliseconds() < 1000) { //Loop until VuMark is detected, timeout after 1 second
            vuMark = vuMarkReader.getVuMark();
            telemetry.addData("Determining VuMark", vuMark);
            telemetry.update();
        }


        //Default to close cryptobox column if VuMark is not detected
        if(vuMark.equals(RelicRecoveryVuMark.UNKNOWN)) vuMark = allianceColor.equals("Blue") ? RelicRecoveryVuMark.LEFT : RelicRecoveryVuMark.RIGHT;

        return vuMark;
    }

    /**
     * Calls turn() with a default of recurse = 1, maxSpeed = turnSpeed
     * It recursively calls itself to improve accuracy and fix overshooting.
     * The speed decreases as the turn nears completion.
     *
     * @param degrees   number of degrees to turn (positive)
     * @param direction Enum Direction.RIGHT or Direction.LEFT
     * @see OpModeBase#turn(double, OpModeBase.Direction, double, int, double)
     * @see ModernRoboticsI2cGyro
     * @see DcMotor
     */
    void turn(double degrees, OpModeBase.Direction direction) {
        if (!opModeIsActive()) return;
        turn(degrees, direction, turnSpeed, 1, 10000);
    }

    /**
     * Calls turn() with a default of recursing once
     * It recursively calls itself to improve accuracy and fix overshooting.
     * The speed decreases as the turn nears completion.
     *
     * @param degrees   number of degrees to turn (positive)
     * @param direction Enum Direction.RIGHT or Direction.LEFT
     * @param maxSpeed  the maximum speed of the robot
     * @see OpModeBase#turn(double, OpModeBase.Direction, double, int, double)
     * @see ModernRoboticsI2cGyro
     * @see DcMotor
     */
    void turn(double degrees, OpModeBase.Direction direction, double maxSpeed) {
        if (!opModeIsActive()) return;
        turn(degrees, direction, maxSpeed, 1, 10000);
    }

    /**
     * This method is used to turn the robot a specific number of degrees in a specific direction.
     * It recursively calls itself to improve accuracy and fix overshooting.
     * The speed decreases as the turn nears completion.
     *
     * @param degrees   number of degrees to turn (positive)
     * @param direction Enum Direction.RIGHT or Direction.LEFT
     * @param maxSpeed  the maximum speed of the robot
     * @param count     the number of times to recurse
     * @see OpModeBase#turn(double, OpModeBase.Direction, double)
     * @see ModernRoboticsI2cGyro
     * @see DcMotor
     */
    void turn(double degrees, OpModeBase.Direction direction, double maxSpeed, int count, double timeout) {
        if (!opModeIsActive()) return; //Necessary because turn method is recursive
        if (direction.equals(OpModeBase.Direction.LEFT)) degrees *= -1; //Negative degree for turning left
        double initialHeading = getIntegratedHeading();
        double targetHeading = initialHeading + degrees; //Turns are relative to current position

        //Change mode because turn() uses motor power PID and not motor position
        motorLeftFront.setMode(RUN_USING_ENCODER);
        motorLeftBack.setMode(RUN_USING_ENCODER);
        motorRightFront.setMode(RUN_USING_ENCODER);
        motorRightBack.setMode(RUN_USING_ENCODER);

        ElapsedTime timer = new ElapsedTime();
        timer.startTime();

        //While target has not been reached, stops robot if target is overshot
        while (((degrees < 0 && getIntegratedHeading() > targetHeading) || (degrees > 0 && getIntegratedHeading() < targetHeading)) && (timer.milliseconds() < timeout) && opModeIsActive()) {
            double currentHeading = getIntegratedHeading();

            double robotSpeed = Range.clip(maxSpeed * (Math.abs(targetHeading - getIntegratedHeading()) / Math.abs(degrees)), .2, maxSpeed);

            motorLeftFront.setPower(degrees < 0 ? robotSpeed : -robotSpeed);
            motorLeftBack.setPower(degrees < 0 ? robotSpeed : -robotSpeed);
            motorRightFront.setPower(degrees < 0 ? -robotSpeed : robotSpeed);
            motorRightBack.setPower(degrees < 0 ? -robotSpeed : robotSpeed);

            telemetry.addData("Heading", getIntegratedHeading());
            telemetry.addData("Distance to turn: ", Math.abs(currentHeading - targetHeading));
            telemetry.addData("Target", targetHeading);
            telemetry.addData("Heading", currentHeading);
            telemetry.addData("Initial Heading", initialHeading);
            telemetry.addData("Power", robotSpeed);
            telemetry.update();
        }

        motorLeftFront.setPower(0);
        motorLeftBack.setPower(0);
        motorRightFront.setPower(0);
        motorRightBack.setPower(0);
        sleep(300);

        telemetry.addData("Distance to turn", Math.abs(getIntegratedHeading() - targetHeading));
        telemetry.addData("Direction", -1 * (int) Math.signum(degrees));
        telemetry.update();

        if (Math.abs(getIntegratedHeading() - targetHeading) > 3 && count > 0) { //If the target was significantly overshot
            //Recurse to correct turn
            turn(Math.abs(getIntegratedHeading() - targetHeading), direction.next(), .2, --count, 2000);
        }
    }

    /**
     * Calls move with a default of maxSpeed = moveSpeedMin, recurse = true, PID = true, timeout = 10000000
     *
     * @param distance the distance in inches to move
     * @see OpModeBase#move(double, Direction, double, boolean, double)
     * @see DcMotor
     * @see ModernRoboticsI2cGyro
     */
    void move(double distance, Direction direction) {
        move(distance, direction, moveSpeedMax, false, 10000);
    }

    /**
     * Moves the robot a specified number of inches.
     * The speed of the robot is defined using a trapazoidal motion profile.
     * It uses the gyroscope to keep the movement straight.
     * It will call turn() at the end to correct heading error.
     *
     * @param distance the distance in inches to move
     * @param maxSpeed the maximum speed at which to run the robot
     * @param recurse  whether or not to call turn()
     * @see OpModeBase#turn
     * @see DcMotor
     * @see ModernRoboticsI2cGyro
     */
    void move(double distance, Direction direction, double maxSpeed, boolean recurse, double timeout) {
        double motorInitial = motorLeftFront.getCurrentPosition();
        double initialHeading = getIntegratedHeading();

        //Change mode because move() uses setTargetPosition()
        motorLeftFront.setMode(RUN_TO_POSITION);
        motorLeftBack.setMode(RUN_TO_POSITION);
        motorRightFront.setMode(RUN_TO_POSITION);
        motorRightBack.setMode(RUN_TO_POSITION);

        if (direction == Direction.FORWARD) {
            distance *= ticksRatioForward;
            motorLeftFront.setTargetPosition((int) (motorLeftFront.getCurrentPosition() + distance));
            motorLeftBack.setTargetPosition((int) (motorLeftBack.getCurrentPosition() + distance));
            motorRightFront.setTargetPosition((int) (motorRightFront.getCurrentPosition() + distance));
            motorRightBack.setTargetPosition((int) (motorRightBack.getCurrentPosition() + distance));
        } else if (direction == Direction.BACKWARD) {
            distance *= ticksRatioForward;
            motorLeftFront.setTargetPosition((int) (motorLeftFront.getCurrentPosition() - distance));
            motorLeftBack.setTargetPosition((int) (motorLeftBack.getCurrentPosition() - distance));
            motorRightFront.setTargetPosition((int) (motorRightFront.getCurrentPosition() - distance));
            motorRightBack.setTargetPosition((int) (motorRightBack.getCurrentPosition() - distance));
        } else if (direction == Direction.LEFT) {
            distance *= ticksRatioStrafe;
            motorLeftFront.setTargetPosition((int) (motorLeftFront.getCurrentPosition() + distance));
            motorLeftBack.setTargetPosition((int) (motorLeftBack.getCurrentPosition() - distance));
            motorRightFront.setTargetPosition((int) (motorRightFront.getCurrentPosition() - distance));
            motorRightBack.setTargetPosition((int) (motorRightBack.getCurrentPosition() + distance));
        } else {
            distance *= ticksRatioStrafe;
            motorLeftFront.setTargetPosition((int) (motorLeftFront.getCurrentPosition() - distance));
            motorLeftBack.setTargetPosition((int) (motorLeftBack.getCurrentPosition() + distance));
            motorRightFront.setTargetPosition((int) (motorRightFront.getCurrentPosition() + distance));
            motorRightBack.setTargetPosition((int) (motorRightBack.getCurrentPosition() - distance));
        }

        double moveSpeed = moveSpeedMin;

        motorLeftFront.setPower(moveSpeed);
        motorLeftBack.setPower(moveSpeed);
        motorRightFront.setPower(moveSpeed);
        motorRightBack.setPower(moveSpeed);

        ElapsedTime timer = new ElapsedTime();
        timer.startTime();

        while ((motorLeftFront.isBusy() && motorLeftBack.isBusy() && motorRightFront.isBusy() && motorRightBack.isBusy()) && timer.milliseconds() < timeout && opModeIsActive()) {
            //Only one encoder target must be reached

            //Trapezoidal motion profile
            if (Math.abs(motorLeftFront.getCurrentPosition() - motorInitial) < 1000) {
                moveSpeed = Math.min(maxSpeed, moveSpeed + .05); //Ramp up motor speed at beginning of move
            } else if (Math.abs(motorLeftFront.getCurrentPosition() - motorLeftFront.getTargetPosition()) < 1000) { //Ramp down motor speed at end of move
                moveSpeed = Math.max(moveSpeedMin, moveSpeed - .03);
            }

            motorLeftFront.setPower(moveSpeed);
            motorLeftBack.setPower(moveSpeed);
            motorRightFront.setPower(moveSpeed);
            motorRightBack.setPower(moveSpeed);

            telemetry.addData("Motor Speed", moveSpeed);
            telemetry.update();
        }

        motorLeftFront.setPower(0);
        motorLeftBack.setPower(0);
        motorRightFront.setPower(0);
        motorRightBack.setPower(0);
        sleep(400);

        //Correct if robot turned during movement
        if (Math.abs(getIntegratedHeading() - initialHeading) > 5 && recurse) {
            turn(Math.abs(getIntegratedHeading() - initialHeading), getIntegratedHeading() < initialHeading ? OpModeBase.Direction.RIGHT : OpModeBase.Direction.LEFT, .2);
        }
    }

    /**
     * Determines the color read by the color sensor.
     * This method converts the RGB values returned from the REV Color / Distance sensor to the HSV color space.
     *
     * @return color: "Red", "Blue", "Unknown"
     */
    String getColor() {
        float[] hsv = {0F, 0F, 0F};

        Color.RGBToHSV(colorSensor.red() * 255, colorSensor.green() * 255, colorSensor.blue() * 255, hsv);

        if (hsv[2] < 10) return "Unknown";
        if ((hsv[0] < 30 || hsv[0] > 340) && hsv[1] > .2) return "Red";
        else if ((hsv[0] > 170 && hsv[0] < 260) && hsv[1] > .2) return "Blue";
        return "Unknown";
    }

    private double getIntegratedHeading() { //https://ftcforum.usfirst.org/forum/ftc-technology/53477-rev-imu-questions?p=53481#post53481
        //IMU is mounted vertically, so the Y axis is used for turning
        double currentHeading = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle;
        double deltaHeading = currentHeading - previousHeading;

        if (deltaHeading < -180) deltaHeading += 360;
        else if (deltaHeading >= 180) deltaHeading -= 360;

        integratedHeading += deltaHeading;
        previousHeading = currentHeading;

        return integratedHeading;
    }
}