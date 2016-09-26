package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

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
    //private ColorSensor colorBottom;
    private OpticalDistanceSensor ods;

    //Variables
    private final double ENCODER_RATIO = 25.8693401577559; //ticks / in

    private float[] hsv = {0F, 0F, 0F};


    public void runOpMode() throws InterruptedException {

        //Hardware Instantiation
        motorLeft1 = hardwareMap.dcMotor.get("left1");
        motorLeft2 = hardwareMap.dcMotor.get("left2");
        motorRight1 = hardwareMap.dcMotor.get("right1");
        motorRight2 = hardwareMap.dcMotor.get("right2");

        motorLeft1.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLeft2.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRight1.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRight2.setDirection(DcMotorSimple.Direction.REVERSE);

        buttonPresserLeft = hardwareMap.servo.get("button_left");
        buttonPresserRight = hardwareMap.servo.get("button_left");

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        colorFront = hardwareMap.colorSensor.get("color_front");
        //colorBottom = hardwareMap.colorSensor.get("color_bottom");
        ods = hardwareMap.opticalDistanceSensor.get("ods");

        //Gyro reset
        gyro.calibrate();

        // Wait while gyro is calibrating
        while (gyro.isCalibrating())  {
            Thread.sleep(50);
            idle();
        }

        telemetry.addData("Gyroscope is calibrated.", "");
        telemetry.update();

        //Motor Reset
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

        waitForStart();

        //Beginning of Actual Code
        move(24);

    }

    public void turn(int degrees) throws InterruptedException { //Positive degree for turning left
        int currentHeading = gyro.getIntegratedZValue();
        int targetHeading = currentHeading + degrees;

        //Change mode because turn() uses motor power and not motor position
        motorLeft1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeft2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if(degrees < 0) {
            while(gyro.getIntegratedZValue() > targetHeading) {
                motorLeft1.setPower(-.5);
                motorLeft2.setPower(-.5);
                motorRight1.setPower(.5);
                motorRight2.setPower(.5);

                telemetry.addData("Heading", gyro.getIntegratedZValue());
                telemetry.update();
                idle();
            }
        } else { //Left
            while (gyro.getIntegratedZValue() < targetHeading) {
                motorLeft1.setPower(.5);
                motorLeft2.setPower(.5);
                motorRight1.setPower(-.5);
                motorRight2.setPower(-.5);

                telemetry.addData("Heading", gyro.getIntegratedZValue());
                telemetry.update();
                idle();
            }
        }

        motorLeft1.setPower(0);
        motorLeft2.setPower(0);
        motorRight1.setPower(0);
        motorRight2.setPower(0);
    }

    public void move(double distance) throws InterruptedException {
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

        motorLeft1.setPower(.5);
        motorLeft2.setPower(.5);
        motorRight1.setPower(.5);
        motorRight2.setPower(.5);

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
}