package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.graphics.Color;
import android.preference.PreferenceManager;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Contains variables and methods used to control the robot. Autonomous and TeleOp classes are subclasses of OpModeBase.
 */

abstract class OpModeBase extends LinearOpMode {
    //*************** Declare Hardware Devices ***************

    //Motors
    DcMotor motorLeft1;
    DcMotor motorLeft2;
    DcMotor motorRight1;
    DcMotor motorRight2;

    DcMotor glyphLift;
    DcMotor relicLift;

    Servo leftGlyphGrabber;
    Servo rightGlyphGrabber;
    Servo arm; //Arm is on left side of robot looking at robot from back
    Servo relicGrabber;
    Servo relicRotator;

    //Sensors
    private BNO055IMU imu;
    private ColorSensor colorSensor; //Color sensor is on left side of arm
    ModernRoboticsI2cRangeSensor range;

    //SharedPreferences
    private SharedPreferences sharedPreferences;

    String allianceColor;
    String location;
    int delay;

    //General Constants
    static final double LEFT_GLYPH_GRABBR_OPEN = .43;
    static final double LEFT_GLYPH_GRABBR_CLOSED = .07;
    static final double RIGHT_GLYPH_GRABBR_OPEN = .23;
    static final double RIGHT_GLYPH_GRABBR_CLOSED = .77;

    static final double COLOR_SENSOR_ARM_IN = 0;
    static final double COLOR_SENSOR_ARM_OUT = .9;

    static final double RELIC_GRABBER_INITIAL = .5;
    static final double RELIC_ROTATOR_INITIAL = .5;

    //Autonomous Specific Configuration
    private double moveSpeedMin = .15;
    double moveSpeedMax = .90;
    private double kP = 0.0;
    private double ticksRatio = 3000 / 34.5; //Ticks / inch

    double turnSpeed = .4; //Speed is ramped up and down

    enum Direction {
        LEFT, RIGHT;

        //Taken from: http://stackoverflow.com/a/17006263
        private static OpModeBase.Direction[] vals = values();

        public OpModeBase.Direction next() {
            return vals[(this.ordinal() + 1) % vals.length];
        }
    }

    //General Variables
    double previousHeading = 0;
    double integratedHeading = 0;

    /**
     * Configures all parts of the robot.
     */
    public void runOpMode(Class<?> className) {
        mapHardware();

        //*************** Configure hardware devices ***************

        //Drive motors
        motorLeft1.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeft2.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRight1.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRight2.setDirection(DcMotorSimple.Direction.FORWARD);

        motorLeft1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //Autonomous methods that need a different mode will set it, RUN_WITHOUT_ENCODER used for teleop
        motorLeft2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorLeft1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //Default is float
        motorLeft2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //Other motors
        glyphLift.setDirection(DcMotorSimple.Direction.FORWARD);
        glyphLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        glyphLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        glyphLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        relicLift.setDirection(DcMotorSimple.Direction.FORWARD);
        relicLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        relicLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        relicLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Servos initialized only during autonomous init period
        if(className.equals(RelicRecoveryAutonomous.class)) initializeServos();

        //Sensors
        if(className.equals(RelicRecoveryAutonomous.class)) initializeIMU(); //IMU not needed for teleop
        colorSensor.enableLed(true);

        //*************** Configure SharedPreferences ***************
        if(className.equals(RelicRecoveryAutonomous.class)) {
            sharedPreferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);
            allianceColor = sharedPreferences.getString("com.qualcomm.ftcrobotcontroller.Autonomous.Color", "null");
            location = sharedPreferences.getString("com.qualcomm.ftcrobotcontroller.Autonomous.Location", "null");
            delay = sharedPreferences.getInt("com.qualcomm.ftcrobotcontroller.Autonomous.Delay", 0);
        }
    }

    private void mapHardware() {
        //Drive Motors
        motorLeft1 = hardwareMap.dcMotor.get("left 1");
        motorLeft2 = hardwareMap.dcMotor.get("left 2");
        motorRight1 = hardwareMap.dcMotor.get("right 1");
        motorRight2 = hardwareMap.dcMotor.get("right 2");

        //Other motors
        glyphLift = hardwareMap.dcMotor.get("glyph lift");
        relicLift = hardwareMap.dcMotor.get("relic lift");

        //Sensors
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        colorSensor = hardwareMap.colorSensor.get("color");
        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");

        //Servos
        leftGlyphGrabber = hardwareMap.servo.get("left grabber");
        rightGlyphGrabber = hardwareMap.servo.get("right grabber");
        arm = hardwareMap.servo.get("arm");
        relicGrabber = hardwareMap.servo.get("relic grabber");
        relicRotator = hardwareMap.servo.get("relic rotator");
    }

    void initializeServos() {
        leftGlyphGrabber.setPosition(LEFT_GLYPH_GRABBR_CLOSED); //Grabbers begin closed to hold the glyph
        rightGlyphGrabber.setPosition(RIGHT_GLYPH_GRABBR_CLOSED);

        relicGrabber.setPosition(RELIC_GRABBER_INITIAL);
        relicRotator.setPosition(RELIC_ROTATOR_INITIAL);


        arm.setPosition(COLOR_SENSOR_ARM_IN);
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
     * @param count the number of times to recurse
     * @see OpModeBase#turn(double, OpModeBase.Direction, double)
     * @see ModernRoboticsI2cGyro
     * @see DcMotor
     */
    void turn(double degrees, OpModeBase.Direction direction, double maxSpeed, int count) {
        if (!opModeIsActive()) return;
        if (direction.equals(OpModeBase.Direction.RIGHT)) degrees *= -1; //Negative degree for turning right
        double initialHeading = getIntegratedHeading();
        double targetHeading = initialHeading + degrees; //Turns are relative to current position
        double robotSpeed = .1;

        //Change mode because turn() uses motor power and not motor position
        motorLeft1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeft2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //While target has not been reached, stops robot if target is overshot
        while (((degrees < 0 && getIntegratedHeading() > targetHeading) || (degrees > 0 && getIntegratedHeading() < targetHeading)) && opModeIsActive()) {
            double currentHeading = getIntegratedHeading();

            if(Math.abs(currentHeading - targetHeading) < 35 && Math.abs(currentHeading - targetHeading) > 15) { //Ramp down motor speed at end of turn, last 15 degrees
                robotSpeed = Math.max(.1, robotSpeed - .08);
            } else if(Math.abs(currentHeading - targetHeading) < 5) { //Ramp down motor speed at end of turn, last 15 degrees
                robotSpeed = .02;
            } else if(Math.abs(currentHeading - initialHeading) < 15) { //Ramp up motor speed at beginning of turn, 1st 15 degrees
                robotSpeed = Math.min(maxSpeed, robotSpeed + .04);
            }

            motorLeft1.setPower(degrees < 0 ? -robotSpeed : robotSpeed);
            motorLeft2.setPower(degrees < 0 ? -robotSpeed : robotSpeed);
            motorRight1.setPower(degrees < 0 ? robotSpeed : -robotSpeed);
            motorRight2.setPower(degrees < 0 ? robotSpeed : -robotSpeed);

            telemetry.addData("Distance to turn: ", Math.abs(currentHeading - targetHeading));
            telemetry.addData("Target", targetHeading);
            telemetry.addData("Heading", currentHeading);
            telemetry.addData("Initial Heading", initialHeading);
            telemetry.addData("Power", robotSpeed);
            telemetry.update();
        }

        motorLeft1.setPower(0);
        motorLeft2.setPower(0);
        motorRight1.setPower(0);
        motorRight2.setPower(0);
        sleep(200);

        telemetry.addData("Distance to turn", Math.abs(getIntegratedHeading() - targetHeading));
        telemetry.addData("Direction", -1 * (int) Math.signum(degrees));
        telemetry.update();

        if (Math.abs(getIntegratedHeading() - targetHeading) > 3 && count > 0) { //If the target was significantly overshot
            //Recurse to correct turn
            turn(Math.abs(getIntegratedHeading() - targetHeading), direction.next(), .1, --count);
        }
    }

    /**
     * Calls move with a default of maxSpeed = moveSpeedMin, recurse = true, PID = true
     * @param distance the distance in inches to move
     * @see OpModeBase#move(double, double, boolean)
     * @see DcMotor
     * @see ModernRoboticsI2cGyro
     */
    void move(double distance) {
        move(distance, moveSpeedMax, true, kP);
    }

    /**
     * Calls move with a default of recurse = true, PID = true
     * @param distance the distance in inches to move
     * @param maxSpeed the maximum speed to run the robot
     * @see OpModeBase#move(double, double, boolean, double)
     * @see DcMotor
     * @see ModernRoboticsI2cGyro
     */
    void move(double distance, double maxSpeed) {
        move(distance, moveSpeedMax, true, kP);
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
        move(distance, moveSpeedMax, recurse, kP);
    }

    /**
     * Moves the robot a specified number of inches.
     * The speed of the robot decreases in a sigmoid fashion as the target is approached.
     * It uses the gyroscope to keep the movement straight.
     * It will call turn() at the end to correct heading eror.
     *
     * @param distance the distance in inches to move
     * @param maxSpeed the maximum speed at which to run the robot
     * @param recurse whether or not to call turn()
     * @see OpModeBase#move(double, double)
     * @see OpModeBase#turn
     * @see DcMotor
     * @see ModernRoboticsI2cGyro
     */
    void move(double distance, double maxSpeed, boolean recurse, double kP) {
        distance *= -1; //Necessary to deal with encoder issues
        double distanceSign = Math.signum(distance); //Necessary for moving backwards
        distance *= ticksRatio;
        double motorInitial = motorLeft1.getCurrentPosition();

        double initialHeading = getIntegratedHeading();

        //Change mode because move() uses setTargetPosition()
        motorLeft1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLeft1.setTargetPosition((int) (motorLeft1.getCurrentPosition() + distance));
        motorLeft2.setTargetPosition((int) (motorLeft2.getCurrentPosition() + distance));
        motorRight1.setTargetPosition((int) (motorRight1.getCurrentPosition() + distance));
        motorRight2.setTargetPosition((int) (motorRight2.getCurrentPosition() + distance));

        double moveSpeed = moveSpeedMin;

        motorLeft1.setPower(moveSpeed);
        motorLeft2.setPower(moveSpeed);
        motorRight1.setPower(moveSpeed);
        motorRight2.setPower(moveSpeed);

        while ((motorLeft1.isBusy() && motorLeft2.isBusy() && motorRight1.isBusy() && motorRight2.isBusy()) && opModeIsActive()) {
            //Only one encoder target must be reached
            double turnError = initialHeading - getIntegratedHeading();
            double headingError = turnError * kP * distanceSign;

            if(Math.abs(motorLeft1.getCurrentPosition() - motorInitial) < 1000) {
                moveSpeed = Math.min(maxSpeed, moveSpeed + .05); //Ramp up motor speed at beginning of move
                telemetry.addData("Ramp up", moveSpeed);
            } else if(Math.abs(motorLeft1.getCurrentPosition() - motorLeft1.getTargetPosition()) < 1000) { //Ramp down motor speed at end of move
                moveSpeed = Math.max(moveSpeedMin, moveSpeed - .05);
                telemetry.addData("Ramp down", moveSpeed);
            }

            double leftPower = Range.clip(moveSpeed - headingError, .2, 1);
            double rightPower = Range.clip(moveSpeed + headingError, .2, 1);

            motorLeft1.setPower(leftPower);
            motorLeft2.setPower(leftPower);
            motorRight1.setPower(rightPower);
            motorRight2.setPower(rightPower);

            telemetry.addData("Left motor power", leftPower);
            telemetry.addData("Right motor power", rightPower);
            telemetry.addData("Current Heading", getIntegratedHeading());
            telemetry.addData("Target Heading", initialHeading);
            telemetry.addData("Heading Error", headingError);
            telemetry.addData("Position", motorLeft1.getCurrentPosition());
            telemetry.addData("Target", motorLeft1.getTargetPosition());
            telemetry.addData("Speed", moveSpeedMin);
            telemetry.update();
        }

        motorLeft1.setPower(0);
        motorLeft2.setPower(0);
        motorRight1.setPower(0);
        motorRight2.setPower(0);
        sleep(400);

        //Correct if robot turned during movement
        if (Math.abs(getIntegratedHeading() - initialHeading) > 5 && recurse) {
            turn(Math.abs(getIntegratedHeading() - initialHeading), getIntegratedHeading() > initialHeading ? OpModeBase.Direction.RIGHT : OpModeBase.Direction.LEFT, .2);
        }
    }

    /**
     * Determines the color read by the color sensor.
     * @return color: "Red", "Blue", "Unknown"
     */
    String getColor() {
        float[] hsv = {0F, 0F, 0F};

        Color.RGBToHSV(colorSensor.red() * 255, colorSensor.green() * 255, colorSensor.blue() * 255, hsv);

        if(hsv[2] < 20) return "Unknown";
        if ((hsv[0] < 30 || hsv[0] > 340) && hsv[1] > .2) return "Red";
        else if ((hsv[0] > 170 && hsv[0] < 260) && hsv[1] > .2) return "Blue";
        return "Unknown";
    }

    double getIntegratedHeading() { //https://ftcforum.usfirst.org/forum/ftc-technology/53477-rev-imu-questions?p=53481#post53481
        double currentHeading = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double deltaHeading = currentHeading - previousHeading;

        if (deltaHeading < -180) deltaHeading += 360;
        else if (deltaHeading >= 180) deltaHeading -= 360;

        integratedHeading += deltaHeading;
        previousHeading = currentHeading;

        return integratedHeading;
    }

    void moveUntilCryptoboxRail(boolean isForward, int railNumber) {
        motorLeft1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeft2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        for(int i = 0; i < railNumber; i++) { //Move until railNumber of cryptobox rails have been detected
            double initialDistance = range.getDistance(DistanceUnit.CM);

            while (Math.abs(range.getDistance(DistanceUnit.CM) - initialDistance) > 10) { //While sensor does not sense a close object (rail)
                motorLeft1.setPower(.2 * (isForward ? 1 : -1));
                motorLeft2.setPower(.2 * (isForward ? 1 : -1));
                motorRight1.setPower(.2 * (isForward ? 1 : -1));
                motorRight2.setPower(.2 * (isForward ? 1 : -1));
            }
            sleep(300); //Move past rail to "zero" distance sensor initial reading for subsequent rails; position glyph intake for turning
        }

        motorLeft1.setPower(0);
        motorLeft2.setPower(0);
        motorRight1.setPower(0);
        motorRight2.setPower(0);
    }
}