package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Run to Position Test", group = "Test Code")
public class RunToPositionTest extends OpModeBase {
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
        motorLeftFront.setTargetPosition(2000);
        motorLeftBack.setTargetPosition(2000);
        motorRightFront.setTargetPosition(2000);
        motorRightBack.setTargetPosition(2000);

        motorLeftFront.setPower(.6);
        motorLeftBack.setPower(.6);
        motorRightFront.setPower(.6);
        motorRightBack.setPower(.6);

        while((motorLeftFront.isBusy() || motorLeftBack.isBusy() || motorRightFront.isBusy() || motorRightBack.isBusy()) && opModeIsActive()) {
            telemetry.addData("Left front position", motorLeftFront.getCurrentPosition());
            telemetry.addData("Left back position", motorLeftBack.getCurrentPosition());
            telemetry.addData("Right front position", motorRightFront.getCurrentPosition());
            telemetry.addData("Right back position", motorRightBack.getCurrentPosition());
            telemetry.update();
            idle();
        }
        sleep(5000);

        motorLeftFront.setPower(0);
        motorLeftBack.setPower(0);
        motorRightFront.setPower(0);
        motorRightBack.setPower(0);
        sleep(5000);
    }
}
