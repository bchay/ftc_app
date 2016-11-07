package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Encoder Test", group = "Test Code")
public class EncoderTest extends OpMode {
    DcMotor motorLeft;
    DcMotor motorRight;

    public void init() {
        motorLeft = hardwareMap.dcMotor.get("left");
        motorRight = hardwareMap.dcMotor.get("right");

        motorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Encoders are reset", "");
        telemetry.update();
    }

    public void loop() {
        telemetry.addData("Encoder Position Left", motorLeft.getCurrentPosition());
        telemetry.addData("Encoder Position Right", motorRight.getCurrentPosition());
    }
}