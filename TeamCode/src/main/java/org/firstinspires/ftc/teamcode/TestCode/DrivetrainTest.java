package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Drivetrain Test", group = "Test Code")
public class DrivetrainTest extends LinearOpMode {

    DcMotor motorLeft1;
    DcMotor motorLeft2;
    DcMotor motorRight1;
    DcMotor motorRight2;


    public void runOpMode() {
        motorLeft1 = hardwareMap.dcMotor.get("left 1");
        motorLeft2 = hardwareMap.dcMotor.get("left 2");
        motorRight1 = hardwareMap.dcMotor.get("right 1");
        motorRight2 = hardwareMap.dcMotor.get("right 2");

        motorLeft1.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeft2.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRight1.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRight2.setDirection(DcMotorSimple.Direction.FORWARD);

        motorLeft1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft1.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Autonomous methods that need RUN_TO_POSITION will set the motors, RUN_USING_ENCODER is required for TeleOp and gyro turn
        motorLeft2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLeft1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //Default is float
        motorLeft2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while(opModeIsActive()) {
            motorLeft1.setPower(gamepad1.left_stick_y);
            motorLeft2.setPower(gamepad1.left_stick_y);
            motorRight1.setPower(gamepad1.right_stick_y);
            motorRight2.setPower(gamepad1.right_stick_y);

            telemetry.addData("Left 1 Current Position", motorLeft1.getCurrentPosition());
            telemetry.addData("Left 2 Current Position", motorLeft2.getCurrentPosition());
            telemetry.addData("Right 1 Current Position", motorRight1.getCurrentPosition());
            telemetry.addData("Right 2 Current Position", motorRight2.getCurrentPosition());

            telemetry.addData("Left 1 Power", motorLeft1.getPower());
            telemetry.addData("Left 2 Power", motorLeft2.getPower());
            telemetry.addData("Right 1 Power", motorRight1.getPower());
            telemetry.addData("Right 2 Power", motorRight2.getPower());
            telemetry.update();
        }
    }

}
