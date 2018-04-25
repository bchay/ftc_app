package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Motor Port Test", group = "Test Code")
public class MotorPortTest extends OpMode {
    DcMotor motor1;
    DcMotor motor2;
    DcMotor motor3;
    DcMotor motor4;

    public void init() {
        motor1 = hardwareMap.dcMotor.get("motor 1"); //right back
        motor2 = hardwareMap.dcMotor.get("motor 2"); //left back
        motor3 = hardwareMap.dcMotor.get("motor 3"); //right front
        motor4 = hardwareMap.dcMotor.get("motor 4"); //left front
    }

    public void loop() {
        motor1.setPower(gamepad1.left_stick_y);
        motor2.setPower(gamepad1.right_stick_y);
        motor3.setPower(gamepad2.left_stick_y);
        motor4.setPower(gamepad2.right_stick_y);
    }
}
