package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Lift Test", group = "Test Code")
public class LiftTest extends LinearOpMode {
    DcMotor lift;

    public void runOpMode() {
        lift = hardwareMap.dcMotor.get("glyph lift");
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while(opModeIsActive()) {
            lift.setPower(gamepad2.left_stick_y);

            telemetry.addData("Encoder", lift.getCurrentPosition());
            telemetry.update();
        }
    }
}
