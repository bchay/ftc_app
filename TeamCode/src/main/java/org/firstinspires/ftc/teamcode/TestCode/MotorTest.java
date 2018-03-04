package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Motor Test", group = "Test Code")
public class MotorTest extends OpMode {
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private DcMotor motor4;
    private DcMotor motor5;
    private DcMotor motor6;
    private DcMotor motor7;
    private DcMotor motor8;

    public void init() {
        motor1 = hardwareMap.dcMotor.get("1"); //left back - reverse
        motor2 = hardwareMap.dcMotor.get("2"); //right back
        motor3 = hardwareMap.dcMotor.get("3"); //left front - reverse
        motor4 = hardwareMap.dcMotor.get("4"); //right front
        motor5 = hardwareMap.dcMotor.get("5"); //right intake
        motor6 = hardwareMap.dcMotor.get("6"); //left intake
        motor7 = hardwareMap.dcMotor.get("7"); //glyph lift
        motor8 = hardwareMap.dcMotor.get("8"); //
    }

    public void loop() {
        if(gamepad1.a) motor1.setPower(1);
        else motor1.setPower(0);

        if(gamepad1.b) motor2.setPower(1);
        else motor2.setPower(0);

        if(gamepad1.x) motor3.setPower(1);
        else motor3.setPower(0);

        if(gamepad1.y) motor4.setPower(1);
        else motor4.setPower(0);

        if(gamepad1.dpad_down) motor5.setPower(1);
        else motor5.setPower(0);

        if(gamepad1.dpad_up) motor6.setPower(1);
        else motor6.setPower(0);

        if(gamepad1.dpad_right) motor7.setPower(1);
        else motor7.setPower(0);

        if(gamepad1.dpad_left) motor8.setPower(1);
        else motor8.setPower(0);

        telemetry.addData("Motor 1 power", motor1.getPower());
        telemetry.addData("Motor 2 power", motor2.getPower());
        telemetry.addData("Motor 3 power", motor3.getPower());
        telemetry.addData("Motor 4 power", motor4.getPower());
        telemetry.addData("Motor 5 power", motor5.getPower());
        telemetry.addData("Motor 6 power", motor6.getPower());
        telemetry.addData("Motor 7 power", motor7.getPower());
        telemetry.addData("Motor 8 power", motor8.getPower());
        telemetry.update();
    }
}
