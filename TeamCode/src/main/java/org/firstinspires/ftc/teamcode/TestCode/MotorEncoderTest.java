package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

@TeleOp(name = "Drivetrain Encoder Test", group = "Test Code")
public class MotorEncoderTest extends OpMode {
    DcMotor motorLeftFront;
    DcMotor motorLeftBack;
    DcMotor motorRightFront;
    DcMotor motorRightBack;

    public void init() {
        motorLeftFront = hardwareMap.dcMotor.get("left front");
        motorLeftBack = hardwareMap.dcMotor.get("left back");
        motorRightFront = hardwareMap.dcMotor.get("right front");
        motorRightBack = hardwareMap.dcMotor.get("right back");

        motorLeftFront.setMode(STOP_AND_RESET_ENCODER);
        motorLeftBack.setMode(STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(STOP_AND_RESET_ENCODER);
        motorRightBack.setMode(STOP_AND_RESET_ENCODER);

        motorLeftFront.setMode(RUN_USING_ENCODER);
        motorLeftBack.setMode(RUN_USING_ENCODER);
        motorRightFront.setMode(RUN_USING_ENCODER);
        motorRightBack.setMode(RUN_USING_ENCODER);
    }

    public void loop() {
        motorLeftFront.setPower(gamepad1.left_stick_y);
        motorLeftBack.setPower(gamepad1.right_stick_y);
        motorRightFront.setPower(gamepad2.left_stick_y);
        motorRightBack.setPower(gamepad2.right_stick_y);

        telemetry.addData("Left Front Power", motorLeftFront.getPower()); //Actually LEFT BACK
        telemetry.addData(" Left Back Power", motorLeftBack.getPower()); //Right front
        telemetry.addData(" Right Front Power", motorRightFront.getPower()); //Right back
        telemetry.addData("Right Back Power", motorRightBack.getPower()); //Left front

        telemetry.addData("Left Front Position", motorLeftFront.getCurrentPosition());
        telemetry.addData(" Left Back Position", motorLeftBack.getCurrentPosition());
        telemetry.addData(" Right Front Position", motorRightFront.getCurrentPosition());
        telemetry.addData("Right Back Position", motorRightBack.getCurrentPosition());
        telemetry.update();
    }
}
