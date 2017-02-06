package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Run Wheels", group = "Test Code")
public class RunWheelsTest extends OpModeBase {
    public void runOpMode() {
        super.runOpMode();

        motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();
        move();
    }

    private void move()  {
        motorLeftFront.setTargetPosition(700);
        motorLeftBack.setTargetPosition(700);
        motorRightFront.setTargetPosition(700);
        motorRightBack.setTargetPosition(700);

        motorLeftFront.setPower(.6);
        while(motorLeftFront.isBusy() && opModeIsActive()) {
            idle();
        }
        motorLeftFront.setPower(0);

        motorLeftBack.setPower(.6);
        while(motorLeftBack.isBusy() && opModeIsActive()) {
            idle();
        }
        motorLeftBack.setPower(0);

        motorRightFront.setPower(.6);
        while(motorRightFront.isBusy() && opModeIsActive()) {
            idle();
        }
        motorRightFront.setPower(0);

        motorRightBack.setPower(.6);
        while(motorRightBack.isBusy() && opModeIsActive()) {
            idle();
        }
        motorRightBack.setPower(0);

        sleep(5000);
    }
}