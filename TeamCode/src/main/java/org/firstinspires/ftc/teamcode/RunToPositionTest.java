package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Run to Position Test", group = "Test Code")
public class RunToPositionTest extends LinearOpMode {
    private DcMotor motorLeft;
    private DcMotor motorRight;

    public void runOpMode() throws InterruptedException {
        motorLeft = hardwareMap.dcMotor.get("left");
        motorRight = hardwareMap.dcMotor.get("left");

        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Encoders reset", "");
        telemetry.update();

        waitForStart();

        move();
    }

    public void move() throws InterruptedException {
        motorLeft.setTargetPosition(5000);
        motorRight.setTargetPosition(5000);

        Thread.sleep(100);

        motorLeft.setPower(.5);
        motorRight.setPower(.5);

        while(motorLeft.isBusy() || motorRight.isBusy()) {
            idle();
        }

        motorLeft.setPower(0);
        motorRight.setPower(0);
    }
}