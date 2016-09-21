package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Autonomous", group = "Autonomous")
public class AutonomousCode extends LinearOpMode {

    private DcMotor motorFrontLeft;
    private DcMotor motorBackLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackRight;

    private ModernRoboticsI2cGyro gyro;

    //Constants
    private final double ENCODER_RATIO = 1;


    public void runOpMode() throws InterruptedException {

        //Variable Instantiation
        motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("backLeft");
        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        motorBackRight = hardwareMap.dcMotor.get("backRight");

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        //Gyro reset
        gyro.calibrate();
        while(gyro.isCalibrating()) {
            telemetry.addData("Gyroscope is being calibrated.", "");
            telemetry.update();
            Thread.sleep(50);
            idle();
        }

        telemetry.addData("Gyroscope is calibrated.", "");
        telemetry.update();

        //Motor Reset
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        //Beginning of Actual Code
        turn(-45);

    }

    public void turn(int degrees) throws InterruptedException { //positive degree for turning right
        int currentHeading = gyro.getIntegratedZValue();
        int targetHeading = currentHeading + degrees;

        telemetry.addData("Current heading: ", gyro.getIntegratedZValue());
        telemetry.addData("Degrees: ", degrees);


        if(degrees < 0) { //Left turn
            while(gyro.getIntegratedZValue() > currentHeading + degrees) {
                motorFrontLeft.setPower(-.5);
                motorBackLeft.setPower(-.5);
                motorFrontRight.setPower(.5);
                motorFrontLeft.setPower(.5);
                telemetry.addData("Distance to turn: ", gyro.getIntegratedZValue() - targetHeading);
                telemetry.update();
                Thread.sleep(50);
                idle();
            }
        } else { //Right turn
            while (gyro.getIntegratedZValue() < currentHeading + degrees) { //Left turn
                motorFrontLeft.setPower(.5);
                motorBackLeft.setPower(.5);
                motorFrontRight.setPower(-.5);
                motorFrontLeft.setPower(-.5);

                telemetry.addData("Distance to turn: ", targetHeading - gyro.getIntegratedZValue());
                telemetry.update();
                Thread.sleep(50);
                idle();
            }
        }

        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
    }

    public void drive (double distance) throws InterruptedException { //distance in inches
        double ticks = distance * ENCODER_RATIO;
        motorFrontLeft.setTargetPosition((int) (motorFrontLeft.getCurrentPosition() + ticks));
        motorBackLeft.setTargetPosition((int) (motorBackLeft.getCurrentPosition() + ticks));
        motorFrontRight.setTargetPosition((int) (motorFrontRight.getCurrentPosition() + ticks));
        motorBackRight.setTargetPosition((int) (motorBackRight.getCurrentPosition() + ticks));

        motorFrontLeft.setPower(.5);
        motorBackLeft.setPower(.5);
        motorFrontRight.setPower(.5);
        motorBackRight.setPower(.5);

        if(motorFrontLeft.isBusy() && motorBackLeft.isBusy() && motorFrontRight.isBusy() && motorBackRight.isBusy()) { //Forward
            idle();
        }

        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
    }
}