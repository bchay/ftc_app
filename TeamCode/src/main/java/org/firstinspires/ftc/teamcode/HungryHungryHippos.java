package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Hungry Hungry Hippos", group = "Demo Bot")
public class HungryHungryHippos extends OpMode {

    private DcMotor motorLeft;
    private DcMotor motorRight;

    @Override
    public void init() {
        motorLeft = hardwareMap.dcMotor.get("left");
        motorRight = hardwareMap.dcMotor.get("right");
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        motorLeft.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void loop() {
        motorLeft.setPower(gamepad1.left_stick_y);
        motorRight.setPower(gamepad1.right_stick_y);
    }
}