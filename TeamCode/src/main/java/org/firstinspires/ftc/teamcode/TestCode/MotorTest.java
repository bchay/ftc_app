package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Motor Test", group = "Test Code")
public class MotorTest extends OpMode {
    private DcMotor motor;

    public void init() {
        motor = hardwareMap.dcMotor.get("motor");
    }

    public void loop() {
        motor.setPower(gamepad1.left_stick_y);
    }
}
