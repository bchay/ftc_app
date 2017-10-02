package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
GAMEPAD MAPPINGS:

Driver One - Movement - Gamepad 1
----------------------------------

Left Joystick: Left drivetrain motors
Right Joystick: Right drivetrain motors

*/

/**
 * This is the Teleop code for the robot.
 */
@TeleOp(name = "Teleop")
public class Teleop extends LinearOpMode { //Teleop is a LinearOpMode so it can extend the same base class as autonomous
    DcMotor motorLeftFront;
    DcMotor motorLeftBack;
    DcMotor motorRightFront;
    DcMotor motorRightBack;

    public void runOpMode() {

        motorLeftFront = hardwareMap.dcMotor.get("left_front");
        motorLeftBack = hardwareMap.dcMotor.get("left_back");
        motorRightFront = hardwareMap.dcMotor.get("right_front");
        motorRightBack = hardwareMap.dcMotor.get("right_back");


        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Autonomous methods that need RUN_TO_POSITION will set the motors, RUN_USING_ENCODER is required for teleop and gyro turn
        motorLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Autonomous methods that need RUN_TO_POSITION will set the motors, RUN_USING_ENCODER is required for teleop and gyro turn
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            motorLeftFront.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x);
            motorLeftBack.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x);
            motorRightFront.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x);
            motorRightBack.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x);

            telemetry.addData("Left Front Motor Power", motorLeftFront.getPower());
            telemetry.addData("Left Back Motor Power", motorRightFront.getPower());
            telemetry.addData("Right Back Motor Power", motorRightFront.getPower());
            telemetry.addData("Right Motor Power", motorRightFront.getPower());

            telemetry.addData("Left Front Encoder", motorLeftFront.getCurrentPosition());
            telemetry.addData("Left Back Encoder", motorLeftBack.getCurrentPosition());
            telemetry.addData("Right Front Encoder", motorRightFront.getCurrentPosition());
            telemetry.addData("Right Back Encoder", motorRightBack.getCurrentPosition());

            telemetry.addData("Voltage: ", this.hardwareMap.voltageSensor.iterator().next().getVoltage());
            telemetry.update();
        }
    }
}