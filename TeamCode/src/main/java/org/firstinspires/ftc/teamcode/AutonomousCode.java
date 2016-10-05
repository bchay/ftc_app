package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

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
    private ColorSensor colorBottom;
    private OpticalDistanceSensor ods;

    //Variables
    private final double ENCODER_RATIO = 166.898634962; //ticks / in
    private final double BUTTON_PRESSER_LEFT_UP = 0;
    private final double BUTTON_PRESSER_RIGHT_UP = 1;
    private final double BUTTON_PRESSER_LEFT_DOWN = 1;
    private final double BUTTON_PRESSER_RIGHT_DOWN = .1;

    private float[] hsv = {0F, 0F, 0F};


    public void runOpMode() throws InterruptedException {

        //Hardware Instantiation
        motorLeft1 = hardwareMap.dcMotor.get("left1");
        motorLeft2 = hardwareMap.dcMotor.get("left2");
        motorRight1 = hardwareMap.dcMotor.get("right1");
        motorRight2 = hardwareMap.dcMotor.get("right2");

        buttonPresserLeft = hardwareMap.servo.get("button_left");
        buttonPresserRight = hardwareMap.servo.get("button_left");

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        colorFront = hardwareMap.colorSensor.get("color_front");
        colorBottom = hardwareMap.colorSensor.get("color_bottom");
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
        Log.i("AUTONOMOUS 6287", "GYRO IS CALIBRATING");
        while (gyro.isCalibrating()) {
            telemetry.addData("Gyroscope is currently calibrating.", "");
            telemetry.update();
            Thread.sleep(50);
            idle();
        }

        telemetry.addData("Gyroscope is calibrated.", "");
        telemetry.update();

        //Enable Color Sensor LEDs
        colorBottom.enableLed(true);
        colorFront.enableLed(false);

        //Change Color Sensor I2C Addresses
        colorFront.setI2cAddress(I2cAddr.create8bit(0x3c));
        colorBottom.setI2cAddress(I2cAddr.create8bit(0x4c));

        telemetry.addData("Ready to Start Program", "");
        telemetry.update();

        waitForStart();

        String allianceColor = "red";

        //Beginning of Actual Code

        turn(-90, .5);

        //Robot begins third tile away from corner vortex wall, wheels touching next full tile next to vortex

        if(allianceColor.equals("red")) {
            //move(44, .5); //initial forward movement
            //turn(90, .1);
            //move(31, .5); //Approach to beacon
        }
    }

    public void turn(int degrees, double maxSpeed) throws InterruptedException { //Positive degree for turning left
        int targetHeading = gyro.getHeading() + degrees;

        //Change mode because turn() uses motor power and not motor position
        motorLeft1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeft2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if(degrees < 0) {
            while(gyro.getIntegratedZValue() > targetHeading) {
                motorLeft1.setPower(-maxSpeed);
                motorLeft2.setPower(-maxSpeed);
                motorRight1.setPower(maxSpeed);
                motorRight2.setPower(maxSpeed);

                telemetry.addData("Distance to turn: ", Math.abs(gyro.getIntegratedZValue() - targetHeading));
                telemetry.addData("Heading", gyro.getIntegratedZValue());
                telemetry.update();
                idle();
            }
        } else { //Left
            while (gyro.getIntegratedZValue() < targetHeading) {
                motorLeft1.setPower(maxSpeed);
                motorLeft2.setPower(maxSpeed);
                motorRight1.setPower(-maxSpeed);
                motorRight2.setPower(-maxSpeed);

                telemetry.addData("Distance to turn: ", Math.abs(gyro.getIntegratedZValue() - targetHeading));
                telemetry.addData("Heading", gyro.getIntegratedZValue());
                telemetry.update();
                idle();
            }
        }

        motorLeft1.setPower(0);
        motorLeft2.setPower(0);
        motorRight1.setPower(0);
        motorRight2.setPower(0);
        idle();
        Thread.sleep(1000);

        /*
        Thread.sleep(2000);
        telemetry.addData("Distance to turn", Math.abs(gyro.getIntegratedZValue() - targetHeading));
        telemetry.addData("Direction", -1 * (int) Math.signum(degrees));
        telemetry.update();

        Thread.sleep(5000);

        if(Math.abs(gyro.getIntegratedZValue() - targetHeading) > 0) {
            turn(Math.abs(gyro.getIntegratedZValue() - targetHeading) * -1 * (int) Math.signum(degrees), .1);
        }
        */
    }

    public void move(double distance, double maxSpeed) throws InterruptedException {
        distance *= ENCODER_RATIO;

        //Change mode because move() uses setTargetPosition()
        motorLeft1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLeft1.setTargetPosition((int) (motorLeft1.getCurrentPosition() + distance));
        motorLeft2.setTargetPosition((int) (motorLeft2.getCurrentPosition() + distance));
        motorRight1.setTargetPosition((int) (motorRight1.getCurrentPosition() + distance));
        motorRight2.setTargetPosition((int) (motorRight2.getCurrentPosition() + distance));

        Thread.sleep(50);

        motorLeft1.setPower(maxSpeed);
        motorLeft2.setPower(maxSpeed);
        motorRight1.setPower(maxSpeed);
        motorRight2.setPower(maxSpeed);

        while(motorLeft1.isBusy() || motorLeft2.isBusy() || motorRight1.isBusy() || motorRight2.isBusy()) {
            telemetry.addData("Gyroscope Heading", gyro.getIntegratedZValue());
            telemetry.update();
            idle();
        }

        motorLeft1.setPower(0);
        motorLeft2.setPower(0);
        motorRight1.setPower(0);
        motorRight2.setPower(0);
    }

    public String getColorName(float[] hsv) {
        if ((hsv[0] < 30 || hsv[0] > 340) && hsv[1] > .2) return "red";
        else if ((hsv[0] > 170 && hsv[0] < 260) && hsv[1] > .2) return "blue";
        return null;
    }
}
