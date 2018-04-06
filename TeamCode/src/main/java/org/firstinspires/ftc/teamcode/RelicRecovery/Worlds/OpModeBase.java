package org.firstinspires.ftc.teamcode.RelicRecovery.Worlds;

import android.graphics.Color;

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
import org.firstinspires.ftc.teamcode.VuMarkReader;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

/**
 * This class contains variables and methods used to control the robot. Autonomous and TeleOp classes are subclasses of OpModeBaseNSR.
 * This class is for the North Super Regional robot for the 2017 - 2018 Relic Recovery FTC season. This class is abstract, and so all
 * implementations must be subclasses of it, rather than an instance of this class.
 */
abstract class OpModeBase extends LinearOpMode {
    //*************** Declare Hardware Devices ***************

    //Motors
    DcMotor motorLeftFront;
    DcMotor motorLeftBack;
    DcMotor motorRightFront;
    DcMotor motorRightBack;

    DcMotor glyphLift;
    DcMotor leftIntake;
    DcMotor rightIntake;

    DcMotor led;

    //Servos
    Servo colorSensorArm; //Arm is on right side of robot looking at robot from back
    Servo colorSensorRotator;

    Servo leftFlipper;
    Servo rightFlipper;

    Servo glyphLever;

    Servo grabber; //Grabber servo controls both sides of grabber attached to flipper

    //Sensors
    BNO055IMU imu;

    ColorSensor colorSensorFront; //Used for reading glyph color
    ColorSensor colorSensorBack;

    //General Constants
    static final double COLOR_SENSOR_ARM_INITIAL = .141;
    static final double COLOR_ROTATOR_INITIAL = .082;

    static final double GLYPH_LEVER_DOWN = .911;
    static final double GLYPH_LEVER_UP = .400;
    static final double GLYPH_LEVER_BACK = .1;


    double LEFT_FLIPPER_UP = .828;
    double LEFT_FLIPPER_PARTIALLY_UP = .698;
    double LEFT_FLIPPER_FLAT = .408; //Up position used when jostling glyphs for alignment
    double LEFT_FLIPPER_DOWN = .408;

    double RIGHT_FLIPPER_UP = .562;
    //No RIGHT_FLIPPER_PARTIALLY_UP
    double RIGHT_FLIPPER_FLAT = .861;
    double RIGHT_FLIPPER_DOWN = .961;

    double GRABBER_OPEN = 0;
    double GRABBER_CLOSED = .4;

    //Autonomous Specific Configuration
    private double moveSpeedMin = .2;
    double moveSpeedMax = .95;
    private double ticksRatioForward = 3000 / 34.5; //Ticks / inch

    private double ticksRatioStrafe = 3000 / 29.5;

    //Values are changed by multiple glyph autonomous programs, these are the initial, default values
    double turnSpeed = .5; //Speed is ramped down as turn proceeds
    double turnSpeedMin = .2;

    enum Direction {
        FORWARD, BACKWARD, LEFT, RIGHT;

        //Taken from http://stackoverflow.com/a/17006263
        private static Direction[] vals = values();

        public Direction next() {
            return vals[(this.ordinal() + 1) % vals.length];
        }
    }

    public enum OpModeType {
        AUTONOMOUS, TELEOP;
    }

    //Autonomous instance variables
    private VuMarkReader vuMarkReader;
    private ElapsedTime time; //Used for sensor reading timeout

    //General Variables
    private double previousHeading = 0;
    private double integratedHeading = 0;

    /**
     * Configures all parts of the robot. This method is called by the subclasses of OpModeBase, which allows all of the various OpModes
     * to run using a single base hardware class.
     * @param opModeType An enum of either AUTONOMOUS or TELEOP that specifies the type of opmode.
     * This is used because configuration is slightly different for each type of opmode.
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

        //Other motors

        glyphLift.setMode(STOP_AND_RESET_ENCODER);
        glyphLift.setMode(RUN_USING_ENCODER);
        glyphLift.setZeroPowerBehavior(BRAKE);

        leftIntake.setZeroPowerBehavior(BRAKE);
        rightIntake.setZeroPowerBehavior(BRAKE);

        led.setPower(-1); //Blue

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

    /**
     * This method maps all of the robot hardware.
     */
    private void mapHardware() {
        //Drive Motors
        motorLeftFront = hardwareMap.dcMotor.get("left front");
        motorLeftBack = hardwareMap.dcMotor.get("left back");
        motorRightFront = hardwareMap.dcMotor.get("right front");
        motorRightBack = hardwareMap.dcMotor.get("right back");

        leftIntake = hardwareMap.dcMotor.get("left intake");
        rightIntake = hardwareMap.dcMotor.get("right intake");

        glyphLift = hardwareMap.dcMotor.get("lift");

        led = hardwareMap.dcMotor.get("led");

        //Sensors
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        colorSensorFront = hardwareMap.colorSensor.get("color front");
        colorSensorBack = hardwareMap.colorSensor.get("color back");

        //Servos
        colorSensorArm = hardwareMap.servo.get("color arm");
        colorSensorRotator = hardwareMap.servo.get("color rotator");

        leftFlipper = hardwareMap.servo.get("left flipper");
        rightFlipper = hardwareMap.servo.get("right flipper");

        glyphLever = hardwareMap.servo.get("glyph lever");

        grabber = hardwareMap.servo.get("grabber");
    }

    /**
     * This method initializes all of the servos.
     * It takes a parameter of the type of opmode, as the initialization is slightly different for autonomous and teleop.
     * @param opModeType An enum representing the type of OpMode.
     */
    void initializeServos(OpModeType opModeType) {
        colorSensorArm.setPosition(COLOR_SENSOR_ARM_INITIAL);
        colorSensorRotator.setPosition(COLOR_ROTATOR_INITIAL);

        leftFlipper.setPosition(LEFT_FLIPPER_DOWN);
        rightFlipper.setPosition(RIGHT_FLIPPER_DOWN);

        glyphLever.setPosition(GLYPH_LEVER_DOWN);
    }

    /**
     * This method initializes the BNO055 IMU that is built into the REV Expansion Hub.
     */
    private void initializeIMU() {
        byte AXIS_MAP_SIGN_BYTE = 0x1; //This is what to write to the AXIS_MAP_SIGN register to negate the z axis

        imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);

        sleep(100); //Changing modes requires a delay before doing anything else

        // Write to the AXIS_MAP_SIGN register
        imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN, AXIS_MAP_SIGN_BYTE & 0x0F);

        //Need to change back into the IMU mode to use the gyro
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.IMU.bVal & 0x0F);

        sleep(100); //Changing modes again requires a delay

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        parameters.mode = BNO055IMU.SensorMode.IMU;

        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }


    //*************** Autonomous Methods ***************

    /**
     * This method displays telemetry until the opmode begins to run.
     * If the opmode is stopped during initialization, the method gracefully exits.
     */
    private void displayInitialTelemetry() {
        while(!isStarted() && !isStopRequested()) { //Display distance telemetry for robot alignment
            telemetry.addData("Ready to start the program.", "");
            telemetry.addData("Encoders", motorLeftFront.getCurrentPosition() + motorLeftBack.getCurrentPosition() + motorRightFront.getCurrentPosition() + motorRightBack.getCurrentPosition());
            telemetry.addData("Heading", getIntegratedHeading());
            telemetry.addData("VuMark", vuMarkReader.getVuMark());
            telemetry.update();
        }
    }

    /**
     * This method hits off the jewel matching the opposing alliance's color.
     * This method has longer delays and slower movements, and is thus more accurate than the faster version.
     * @param allianceColor  A string corresponding to the alliance color.
     * This must be equal to either "Blue" or "Red".
     */
    void hitJewel(String allianceColor) {
        colorSensorArm.setPosition(.495); //Move arm down slightly
        sleep(1300);

        colorSensorRotator.setPosition(.2); //Move rotator to center between jewels
        sleep(2000);

        colorSensorArm.setPosition(0); //Move arm down between jewels
        sleep(1200);

        //Knock off jewel of opposing alliance color
        time.reset();
        while(getColor().equals("Unknown") && opModeIsActive() && time.milliseconds() < 5000) { //Timeout at 5 second
            telemetry.addData("Color", "Unknown");
            telemetry.update();
        }

        //Color sensor reads left jewel
        telemetry.addData("Color", getColor());
        telemetry.update();

        //Color sensor is facing right jewel
        if(getColor().equals(allianceColor) && !getColor().equals("Unknown")) {
             colorSensorRotator.setPosition(.7); //Hit left jewel
            sleep(1000);
        } else if(!getColor().equals("Unknown")) { //Color is still detected, is opposing alliance's color
            colorSensorRotator.setPosition(0); //Hit right jewel
            sleep(1000);
        }

        colorSensorArm.setPosition(COLOR_SENSOR_ARM_INITIAL);
        sleep(1000);

        colorSensorRotator.setPosition(COLOR_ROTATOR_INITIAL);
        sleep(1000);
    }

    /**
     * This method reads the VuMark using Vuforia and the phone's camera.
     * This method times out after one second. If the VuMark has not been determined, it defaults to the close cryptobox column.
     * @param allianceColor The alliance color of the robot. This must be either "Blue" or "Red".
     * @return The RelicRecoveryVuMark corresponding to the read VuMark.
     * If the VuMark cannot be determined, it defaults to the closest column for the alliance color.
     * @see RelicRecoveryVuMark
     */
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

    void flipGlyph() {
        //Move intake down by spinning wheels
        colorSensorArm.setPosition(.515); //Move arm down slightly so that the intake can move down
        sleep(1000);

        leftIntake.setPower(1);
        rightIntake.setPower(-1);
        sleep(1200);

        leftIntake.setPower(0);
        rightIntake.setPower(0);

        //Slowly move flipper up tp deposit glyph into cryptobox
        while(opModeIsActive() && Math.abs(leftFlipper.getPosition() - LEFT_FLIPPER_UP) > .01) {
            leftFlipper.setPosition(Range.clip(leftFlipper.getPosition() + .008, 0, LEFT_FLIPPER_UP));
            rightFlipper.setPosition(Range.clip(rightFlipper.getPosition() - .008, RIGHT_FLIPPER_UP, 1));
        }
        leftFlipper.setPosition(LEFT_FLIPPER_UP); //Ensure that flipper is fully up because of Math.abs threshold
        rightFlipper.setPosition(RIGHT_FLIPPER_UP);

        move(4, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up

        leftFlipper.setPosition(LEFT_FLIPPER_DOWN); //Move flipper into robot before ramming back into glyph
        rightFlipper.setPosition(RIGHT_FLIPPER_DOWN);
        sleep(500);

        move(7, Direction.BACKWARD, moveSpeedMax, false, 1000); //Hit glyph again, pushing it into cryptobox
        move(5, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up
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
    void turn(double degrees, Direction direction) {
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
    void turn(double degrees, Direction direction, double maxSpeed) {
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
    void turn(double degrees, Direction direction, double maxSpeed, int count, double timeout) {
        if (!opModeIsActive()) return; //Necessary because turn method is recursive
        if (direction.equals(Direction.RIGHT)) degrees *= -1; //Negative degree for turning left
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

            double robotSpeed = Range.clip(maxSpeed * (Math.abs(targetHeading - getIntegratedHeading()) / Math.abs(degrees)), turnSpeedMin, maxSpeed);

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
        } else if (direction == Direction.RIGHT) {
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
            turn(Math.abs(getIntegratedHeading() - initialHeading), getIntegratedHeading() < initialHeading ? Direction.RIGHT : Direction.LEFT, .2);
        }
    }

    /**
     * Determines the color read by the color sensor.
     * This method converts the RGB values returned from the REV Color / Distance sensor to the HSV color space.
     *
     * @return color: "Red", "Blue", "Unknown"
     */
    String getColor() {
        return "Blue";
    }

    /**
     * This method returns a value of the Z axis of the REV Expansion Hub IMU.
     * It transforms the value from (-180, 180) to (-inf, inf).
     * This code was taken and modified slightly from https://ftcforum.usfirst.org/forum/ftc-technology/53477-rev-imu-questions?p=53481#post53481.
     * @return The integrated heading on the interval (-inf, inf).
     */
    private double getIntegratedHeading() {
        //IMU is mounted vertically, so the Y axis is used for turning
        double currentHeading = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        double deltaHeading = currentHeading - previousHeading;

        if (deltaHeading < -180) deltaHeading += 360;
        else if (deltaHeading >= 180) deltaHeading -= 360;

        integratedHeading += deltaHeading;
        previousHeading = currentHeading;

        return integratedHeading;
    }
}