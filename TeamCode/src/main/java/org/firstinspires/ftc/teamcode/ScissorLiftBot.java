package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Scissor Lift Bot", group = "Teleop")
public class ScissorLiftBot extends OpMode {

    private DcMotor motorLeft;
    private DcMotor motorRight1;
    private DcMotor motorRight2;

    @Override
    public void init() {
        motorLeft = hardwareMap.dcMotor.get("left");
        motorRight1 = hardwareMap.dcMotor.get("right1");
        motorRight2 = hardwareMap.dcMotor.get("right2");

        motorLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        motorLeft.setPower(gamepad1.left_stick_y);
        motorRight1.setPower(gamepad1.left_stick_y);
        motorRight2.setPower(gamepad1.left_stick_y);
    }
}