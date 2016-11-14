package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Motor Test", group = "Test Code")
public class MotorTest extends OpMode {
    DcMotor left;
    DcMotor right;

    DcMotor lift;
    DcMotor shooter;


    public void init() {
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");

        lift = hardwareMap.dcMotor.get("ball_lift");
        shooter = hardwareMap.dcMotor.get("ball_shooter");
    }

    public void loop() {
        left.setPower(1);
        right.setPower(1);

        lift.setPower(1);
        shooter.setPower(.5);

        telemetry.addData("Left motor power", left.getPower());
        telemetry.addData("Right motor power", right.getPower());

        telemetry.addData("Lift motor power", lift.getPower());
        telemetry.addData("Shooter power", shooter.getPower());
    }
}
