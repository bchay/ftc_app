package org.firstinspires.ftc.teamcode.VelocityVortex;

import android.content.SharedPreferences;
import android.graphics.Color;
import android.preference.PreferenceManager;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * Contains variables and methods used to control the robot.
 */
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
    ModernRoboticsI2cGyro gyro;
    private ColorSensor colorSensor;
    OpticalDistanceSensor odsLine;
    OpticalDistanceSensor odsBall;
    TouchSensor touchSensorBottom;

    //Variables
    final double BUTTON_PRESSER_IN = .6;
    final double BUTTON_PRESSER_OUT = 0;

    final double BALL_STOP_UP = 0;
    final double BALL_STOP_BLOCKED = .63;

    //SharedPreferences
    private SharedPreferences sharedPreferences;
    Direction moveDirection;

    String allianceColor;
    private String location;
    private int delay;

    //Autonomous Specific Configuration
    double moveSpeed = .65;
    double kP = .048; //Set for 2218 ticks
    double ticksRatio = 50.1459;

    double turnSpeed = .3;
    private double turnSlowdown = .1;

    private double odsEdge = .24; //odsLine.getRawLightDetected()
    private double odsBallThreshold = .02; //odsLine.getRawLightDetected()

    enum Direction {
        LEFT, RIGHT;

        //Taken from: http://stackoverflow.com/a/17006263
        private static Direction[] vals = values();

        public Direction next() {
            return vals[(this.ordinal() + 1) % vals.length];
        }
    }

    //MecanumTeleop specific configuration
    double motorMax = 1;


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

        shooter = hardwareMap.dcMotor.get("shooter");
        intake = hardwareMap.dcMotor.get("intake");

        //Button Servos
        buttonPresser = hardwareMap.servo.get("button_presser");
        ballStop = hardwareMap.servo.get("ball_stop");

        //Sensor Declaration
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        colorSensor = hardwareMap.colorSensor.get("color");
        odsLine = hardwareMap.opticalDistanceSensor.get("ods_line");
        odsBall = hardwareMap.opticalDistanceSensor.get("ods_ball");
        touchSensorBottom = hardwareMap.touchSensor.get("touch_bottom");

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

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Autonomous methods that need RUN_TO_POSITION will set the motors, RUN_USING_ENCODER is required for teleop and gyro turn
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        //Servos
        buttonPresser.setPosition(BUTTON_PRESSER_IN);
        ballStop.setPosition(BALL_STOP_BLOCKED);

        //Sensors

        //Gyro
        gyro.calibrate();
        while(gyro.isCalibrating() && !isStopRequested()) idle();
        telemetry.addData("Gyroscope calibrated", gyro.getIntegratedZValue());
        telemetry.update();

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
        telemetry.addData("Heading", gyro.getIntegratedZValue());
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
     * @see OpModeBase#turn(double, Direction, double, int)
     * @see ModernRoboticsI2cGyro
     * @see DcMotor
     */
    void turn(double degrees, Direction direction) {
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
     * @see OpModeBase#turn(double, Direction, double, int)
     * @see ModernRoboticsI2cGyro
     * @see DcMotor
     */
    void turn(double degrees, Direction direction, double maxSpeed) {
        if (!opModeIsActive()) return;
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
     * @see OpModeBase#turn(double, Direction, double)
     * @see ModernRoboticsI2cGyro
     * @see DcMotor
     */
    void turn(double degrees, Direction direction, double maxSpeed, int count) {
        if (!opModeIsActive()) return;
        if (direction.equals(Direction.RIGHT)) degrees *= -1; //Negative degree for turning right
        double targetHeading = gyro.getIntegratedZValue() + degrees; //Turns are relative to current position

        //Change mode because turn() uses motor power and not motor position
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (degrees < 0) {
            while (gyro.getIntegratedZValue() > targetHeading && opModeIsActive()) {
                motorLeftFront.setPower(Range.clip(maxSpeed * (Math.abs(gyro.getIntegratedZValue() - targetHeading) / Math.abs(degrees)), turnSlowdown, maxSpeed));
                motorLeftBack.setPower(Range.clip(maxSpeed * (Math.abs(gyro.getIntegratedZValue() - targetHeading) / Math.abs(degrees)), turnSlowdown, maxSpeed));
                motorRightFront.setPower(Range.clip(-maxSpeed * (Math.abs(gyro.getIntegratedZValue() - targetHeading) / Math.abs(degrees)), -maxSpeed, -turnSlowdown));
                motorRightBack.setPower(Range.clip(-maxSpeed * (Math.abs(gyro.getIntegratedZValue() - targetHeading) / Math.abs(degrees)), -maxSpeed, -turnSlowdown));

                telemetry.addData("Distance to turn: ", Math.abs(gyro.getIntegratedZValue() - targetHeading));
                telemetry.addData("Target", targetHeading);
                telemetry.addData("Heading", gyro.getIntegratedZValue());
                telemetry.update();
                idle();
            }
        } else { //Left
            while (gyro.getIntegratedZValue() < targetHeading && opModeIsActive()) {
                motorLeftFront.setPower(Range.clip(-maxSpeed * (Math.abs(gyro.getIntegratedZValue() - targetHeading) / Math.abs(degrees)), -maxSpeed, -turnSlowdown));
                motorLeftBack.setPower(Range.clip(-maxSpeed * (Math.abs(gyro.getIntegratedZValue() - targetHeading) / Math.abs(degrees)), -maxSpeed, -turnSlowdown));
                motorRightFront.setPower(Range.clip(maxSpeed * (Math.abs(gyro.getIntegratedZValue() - targetHeading) / Math.abs(degrees)), turnSlowdown, maxSpeed));
                motorRightBack.setPower(Range.clip(maxSpeed * (Math.abs(gyro.getIntegratedZValue() - targetHeading) / Math.abs(degrees)), turnSlowdown, maxSpeed));

                telemetry.addData("Distance to turn: ", Math.abs(gyro.getIntegratedZValue() - targetHeading));
                telemetry.addData("Target", targetHeading);
                telemetry.addData("Heading", gyro.getIntegratedZValue());
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

        if (Math.abs(gyro.getIntegratedZValue() - targetHeading) > 0 && count > 0) {
            //Recurse to correct turn
            turn(Math.abs(gyro.getIntegratedZValue() - targetHeading), direction.next(), .1, --count);
        }
    }


    /**
     * Calls move with a default of maxSpeed = moveSpeed, recurse = true, PID = true, a = .8, b = .2
     * a and b tuned for 48 inches
     * @param distance the distance in inches to move
     * @see OpModeBase#move(double, double, boolean, double, double, double)
     * @see DcMotor
     * @see ModernRoboticsI2cGyro
     */
    void move(double distance) {
        move(distance, moveSpeed, true, kP, .8, .2);
    }

    /**
     * Calls move with a default of recurse = true, PID = true, a = .8, b = .2
     * a and b tuned for 48 inches
     * @param distance the distance in inches to move
     * @param maxSpeed the maximum speed to run the robot
     * @see OpModeBase#move(double, double, boolean, double, double, double)
     * @see DcMotor
     * @see ModernRoboticsI2cGyro
     */
    void move(double distance, double maxSpeed) {
        move(distance, maxSpeed, true, kP, .8, .2);
    }

    /**
     * Calls move with a default of a = .8, b = .2
     * a and b tuned for 48 inches
     * @param distance the distance in inches to move
     * @param maxSpeed the maximum speed to run the robot
     * @param recurse whether or not to call turn()
     * @see OpModeBase#move(double, double, boolean, double, double, double)
     * @see DcMotor
     * @see ModernRoboticsI2cGyro
     */
    void move(double distance, double maxSpeed, boolean recurse) {
        move(distance, maxSpeed, recurse, kP, .8, .2);
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
     * @param a value used for speed reduction
     * @param b value used for speed reduction
     * @see OpModeBase#move(double, double)
     * @see OpModeBase#turn
     * @see DcMotor
     * @see ModernRoboticsI2cGyro
     * @see OpModeBase#calculateSpeed(double, double, double, double)
     */
    void move(double distance, double maxSpeed, boolean recurse, double kP, double a, double b) {
        double distanceSign = Math.signum(distance); //Necessary for moving backwards
        distance *= ticksRatio;
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
            double turnError = initialHeading - gyro.getIntegratedZValue();
            double headingError = turnError * kP * distanceSign;
            double leftPower = maxSpeed - headingError;
            double rightPower = maxSpeed + headingError;

            double reducedLeftPower = Range.clip(calculateSpeed(leftPower, Math.abs(distance), a, b), .1, 1);
            double reducedRightPower = Range.clip(calculateSpeed(rightPower, Math.abs(distance), a, b), .1, 1);

            motorLeftFront.setPower(reducedLeftPower);
            motorLeftBack.setPower(reducedLeftPower);
            motorRightFront.setPower(reducedRightPower);
            motorRightBack.setPower(reducedRightPower);

            telemetry.addData("Left motor power", reducedLeftPower);
            telemetry.addData("Right motor power", reducedRightPower);
            telemetry.addData("Current Heading", gyro.getIntegratedZValue());
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
        if (Math.abs(gyro.getIntegratedZValue() - initialHeading) > 0 && recurse) {
            turn(Math.abs(gyro.getIntegratedZValue() - initialHeading), gyro.getIntegratedZValue() > initialHeading ? Direction.RIGHT : Direction.LEFT, .1);
        }
    }

    /**
     * Drives with specified power until the ODS reaches a white line.
     * Uses raw light detected.
     * ODS averages detected values.
     * Does not use PID, the robot instead rides along the wall.
     *
     * @param leftPower power for the left side of the robot
     * @param rightPower power for the right side of the robot
     * @see OpticalDistanceSensor
     * @see OpticalDistanceSensor#getRawLightDetected()
     * @see OpModeBase#odsEdge
     * @see DcMotor
     */
    void driveToWhiteLine(double leftPower, double rightPower) {
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (odsLine.getRawLightDetected() < odsEdge && opModeIsActive()) { //ODS averages values
            motorLeftFront.setPower(leftPower); //No PID, rides along side of wall
            motorLeftBack.setPower(leftPower);
            motorRightFront.setPower(rightPower);
            motorRightBack.setPower(rightPower);

            telemetry.addData("Left Motor Power", motorLeftFront.getPower());
            telemetry.addData("Right Motor Power", motorRightFront.getPower());
            telemetry.addData("ODS Reading", odsLine.getRawLightDetected());
            telemetry.update();
            idle();
        }

        motorLeftFront.setPower(0);
        motorLeftBack.setPower(0);
        motorRightFront.setPower(0);
        motorRightBack.setPower(0);
        sleep(600);
    }

    /**
     * Determines the name of the color sensed by the ColorSensor.
     *
     * @return name of color sensed: Red, Blue, Undefined
     * @see ColorSensor
     */
    String getColorName() {
        float[] hsv = {0F, 0F, 0F};
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsv);

        if ((hsv[0] < 30 || hsv[0] > 280)) return "Red";
        else if ((hsv[0] > 150 && hsv[0] < 270) && hsv[1] > .2) return "Blue";
        return "Undefined";
    }

    /**
     * Shoots two particles, sets timeout to 5 seconds
     * Uses ods to determine when particle is loaded.
     * @see OpModeBase#shoot(int)
     * @see DcMotor
     * @see OpticalDistanceSensor
     */
    public void shoot() {
        shoot(4000);
    }

    /**
     * Shoots two particles.
     * Uses ods to determine when particle is loaded.
     * @see OpModeBase#shoot()
     * @see DcMotor
     * @see OpticalDistanceSensor
     */
    void shoot(int timeout) {
        ballStop.setPosition(BALL_STOP_BLOCKED);
        shooter.setPower(1);

        long startTime = System.currentTimeMillis();
        while(opModeIsActive() && odsBallDetected() && System.currentTimeMillis() - startTime < 2000) { //Shoot first ball, 2 second timeout
            idle();
        }

        shooter.setPower(0); //First ball has been shot

        ballStop.setPosition(BALL_STOP_UP);
        sleep(200);

        intake.setPower(1);

        //Second ball
        startTime = System.currentTimeMillis();
        while(opModeIsActive() && !odsBallDetected() && System.currentTimeMillis() - startTime < timeout) { //While second ball isn't loaded, times out after specified time
            telemetry.addData("Time", System.currentTimeMillis() - startTime < timeout);
            idle();
        }

        intake.setPower(0);

        shooter.setPower(1); //Shoot second ball after it is loaded
        sleep(1300);

        shooter.setPower(0);
        ballStop.setPosition(BALL_STOP_BLOCKED);

        int rotations = shooter.getCurrentPosition() / 2500 + 1; //Moves shooter to downward position

        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shooter.setTargetPosition(rotations * 2500);
        shooter.setPower(1);

        while(shooter.isBusy() && opModeIsActive()) {
            telemetry.addData("Position", shooter.getCurrentPosition());
            telemetry.addData("Target", shooter.getTargetPosition());
            telemetry.addData("Rotations", rotations);
            telemetry.update();
            idle();
        }
        shooter.setPower(0);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep(200);
    }


    /**
     * Reduces a given speed using a transformed sigmoid.
     * Reduction is based upon the distance left to move.
     * Uses motorFrontLeft to calculate speed reduction
     * An a value of 0 will return speed
     * See https://www.desmos.com/calculator/bhutcuv6ea
     *
     * @param speed the speed
     * @param distance the total distance of the movement
     * @return the speed reduced based on the point in the movement
     * @see DcMotor
     * @see DcMotor#getCurrentPosition()
     * @see DcMotor#getTargetPosition()
     */
    private double calculateSpeed(double speed, double distance, double a, double b) {
        double numerator = -1 * a * speed;
        double regression = (19.869 / distance) + 0.0029571;

        double x = Math.abs(distance - (Math.abs(motorLeftFront.getTargetPosition() - motorLeftFront.getCurrentPosition())));
        double shift = .6 * distance;
        double reduction = x - shift;

        double exponent = -1 * b * regression * reduction;
        double denominator = 1 + Math.exp(exponent);
        return (numerator / denominator) + speed;
    }

    boolean odsBallDetected() {
        return odsBall.getRawLightDetected() > odsBallThreshold;
    }

    /**
     * Returns the delay specified in the autonomous SharedPreferences menu.
     * @return delay
     */
    public int getDelay() {
        return delay;
    }
}