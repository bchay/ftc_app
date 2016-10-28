package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;
import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import java.io.InterruptedIOException;

@Autonomous(name = "Autonomous", group = "Main Code")
public class AutonomousCode extends LinearOpMode {

    //Hardware Declaration
    private DcMotor motorLeft1;
    private DcMotor motorLeft2;
    private DcMotor motorRight1;
    private DcMotor motorRight2;

    private Servo buttonPresserLeft;
    private Servo buttonPresserRight;

    private ModernRoboticsI2cGyro gyro;
    private ColorSensor colorFront;
    private OpticalDistanceSensor ods;

    //Variables
    private final double ENCODER_RATIO = 89.4575644937; //ticks / in
    private final double BUTTON_PRESSER_LEFT_UP = 1;
    private final double BUTTON_PRESSER_RIGHT_UP = 1;
    private final double BUTTON_PRESSER_LEFT_DOWN = .3;
    private final double BUTTON_PRESSER_RIGHT_DOWN = .7;

    private String turnDirection;
    private double moveSpeed = .3;
    private double turnSpeed = .2;

    private float[] hsv = {0F, 0F, 0F};

    SharedPreferences sharedPreferences;

    public void runOpMode() {

        sharedPreferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);

        //Hardware Instantiation
        motorLeft1 = hardwareMap.dcMotor.get("left1");
        motorLeft2 = hardwareMap.dcMotor.get("left2");
        motorRight1 = hardwareMap.dcMotor.get("right1");
        motorRight2 = hardwareMap.dcMotor.get("right2");

        buttonPresserLeft = hardwareMap.servo.get("button_left");
        buttonPresserRight = hardwareMap.servo.get("button_left");

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        colorFront = hardwareMap.colorSensor.get("color_front");
        ods = hardwareMap.opticalDistanceSensor.get("ods");

        //Position Servoes
        buttonPresserLeft.setPosition(BUTTON_PRESSER_LEFT_UP);
        buttonPresserRight.setPosition(BUTTON_PRESSER_RIGHT_UP);

        //Motor Direction
        motorLeft1.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLeft2.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRight1.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRight2.setDirection(DcMotorSimple.Direction.REVERSE);

        //Motor RunMode
        motorLeft1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Encoders are reset", "");
        telemetry.update();

        //Gyro Calibration
        gyro.calibrate();

        //Wait while gyro is calibrating
        while (gyro.isCalibrating()) {
            telemetry.addData("Gyroscope is currently calibrating.", "");
            telemetry.update();
            sleep(50);
        }

        telemetry.addData("Gyroscope is calibrated.", "");
        telemetry.update();

        //Disable Color Sensor LED
        colorFront.enableLed(false);

        String allianceColor = sharedPreferences.getString("com.qualcomm.ftcrobotcontroller.Autonomous.Color", "null");

        telemetry.addData("Ready to Start Program", "");
        telemetry.update();

        waitForStart();



        //Beginning of Actual Code

        //Robot begins third tile away from corner vortex wall, wheels touching next full tile next to vortex
        if(allianceColor.equals("Blue")) {
            turnDirection = "right";
        } else turnDirection = "left";

        move(24, moveSpeed);
        turn(55, turnDirection, turnSpeed);
        move(20, .3);
        driveToWhiteLine(.5);
        turn(35, turnDirection, turnSpeed);
        move(7, moveSpeed);
    }

    public void turn(int degrees, String direction, double maxSpeed) {
        if(direction.equals("right")) degrees *= -1; //Negative degree for turning right
        int targetHeading = gyro.getIntegratedZValue() + degrees;

        //Change mode because turn() uses motor power and not motor position
        motorLeft1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeft2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if(degrees < 0) {
            while(gyro.getIntegratedZValue() > targetHeading && opModeIsActive()) {
                motorLeft1.setPower(-maxSpeed);
                motorLeft2.setPower(-maxSpeed);
                motorRight1.setPower(maxSpeed);
                motorRight2.setPower(maxSpeed);

                telemetry.addData("Distance to turn: ", Math.abs(gyro.getIntegratedZValue() - targetHeading));
                telemetry.addData("Heading", gyro.getIntegratedZValue());
                telemetry.update();
            }
        } else { //Left
            while (gyro.getIntegratedZValue() < targetHeading && opModeIsActive()) {
                motorLeft1.setPower(maxSpeed);
                motorLeft2.setPower(maxSpeed);
                motorRight1.setPower(-maxSpeed);
                motorRight2.setPower(-maxSpeed);

                telemetry.addData("Distance to turn: ", Math.abs(gyro.getIntegratedZValue() - targetHeading));
                telemetry.addData("Heading", gyro.getIntegratedZValue());
                telemetry.update();
            }
        }

        motorLeft1.setPower(0);
        motorLeft2.setPower(0);
        motorRight1.setPower(0);
        motorRight2.setPower(0);
        sleep((maxSpeed < .2) ? 300 : 1000); //Wait for less time for lower powers

        telemetry.addData("Distance to turn", Math.abs(gyro.getIntegratedZValue() - targetHeading));
        telemetry.addData("Direction", -1 * (int) Math.signum(degrees));
        telemetry.update();

        if(Math.abs(gyro.getIntegratedZValue() - targetHeading) > 0) {
            //Recurse to correct turn
            turn(Math.abs(gyro.getIntegratedZValue() - targetHeading), direction.equals("right") ? "left" : "right", .1);
        }
    }

    public void move(double distance, double maxSpeed) {
        distance *= ENCODER_RATIO;
        int initialHeading = gyro.getIntegratedZValue();

        //Change mode because move() uses setTargetPosition()
        motorLeft1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLeft1.setTargetPosition((int) (motorLeft1.getCurrentPosition() + distance));
        motorLeft2.setTargetPosition((int) (motorLeft2.getCurrentPosition() + distance));
        motorRight1.setTargetPosition((int) (motorRight1.getCurrentPosition() + distance));
        motorRight2.setTargetPosition((int) (motorRight2.getCurrentPosition() + distance));

        sleep(100);

        motorLeft1.setPower(maxSpeed);
        motorLeft2.setPower(maxSpeed);
        motorRight1.setPower(maxSpeed);
        motorRight2.setPower(maxSpeed);

        while(motorLeft1.isBusy() && motorLeft2.isBusy() && motorRight1.isBusy() && motorRight2.isBusy() && opModeIsActive()) {
            //One encoder target must be reached
            telemetry.addData("Gyroscope Heading", gyro.getIntegratedZValue());
            telemetry.addData("Left 1 Distance", Math.abs(motorLeft1.getCurrentPosition() - motorLeft1.getTargetPosition()));
            telemetry.addData("Left 2 Distance", Math.abs(motorLeft2.getCurrentPosition() - motorLeft2.getTargetPosition()));
            telemetry.addData("Right 1 Distance", Math.abs(motorRight1.getCurrentPosition() - motorRight1.getTargetPosition()));
            telemetry.addData("Right 2 Distance", Math.abs(motorRight2.getCurrentPosition() - motorRight2.getTargetPosition()));
            telemetry.update();
        }

        motorLeft1.setPower(0);
        motorLeft2.setPower(0);
        motorRight1.setPower(0);
        motorRight2.setPower(0);
        sleep(300);
        turn(Math.abs(gyro.getIntegratedZValue() - initialHeading), gyro.getIntegratedZValue() > initialHeading ? "right" : "left", .2); //TODO: Check for accuracy
    }

    public void driveToWhiteLine(double power) {
        //Uses encoders for PID, no target for RUN_TO_POSITION
        motorLeft1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeft2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(ods.getRawLightDetected() < .60 && opModeIsActive()) { //.8-.9 is white, ODS averages values it sees
            motorLeft1.setPower(power);
            motorLeft2.setPower(power);
            motorRight1.setPower(power);
            motorRight2.setPower(power);
            telemetry.addData("ODS Reading", ods.getRawLightDetected());
            Log.d("ODS DATA", String.valueOf(ods.getRawLightDetected()));
            telemetry.update();
        }

        motorLeft1.setPower(0);
        motorLeft2.setPower(0);
        motorRight1.setPower(0);
        motorRight2.setPower(0);
        sleep(100);
    }

    public String getColorName(float[] hsv) {
        if ((hsv[0] < 30 || hsv[0] > 340) && hsv[1] > .2) return "red";
        else if ((hsv[0] > 170 && hsv[0] < 260) && hsv[1] > .2) return "blue";
        return "undefined";
    }
}
