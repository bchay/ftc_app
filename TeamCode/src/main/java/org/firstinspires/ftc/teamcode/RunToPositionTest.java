package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Run to Position Test", group = "Test Code")
public class RunToPositionTest extends OpModeBase {
    public void runOpMode() {
        super.runOpMode();

        waitForStart();

        move();
    }

    private void move()  {
        motorLeftFront.setTargetPosition(4000);
        motorLeftBack.setTargetPosition(4000);
        motorRightFront.setTargetPosition(4000);
        motorRightBack.setTargetPosition(4000);

        motorLeftFront.setPower(.5);
        motorLeftBack.setPower(.5);
        motorRightFront.setPower(.5);
        motorRightBack.setPower(.5);

        while(motorLeftFront.isBusy() && motorLeftBack.isBusy() && motorRightFront.isBusy() && motorRightBack.isBusy() && opModeIsActive()) {
            idle();
        }

        motorLeftFront.setPower(0);
        motorLeftBack.setPower(0);
        motorRightFront.setPower(0);
        motorRightBack.setPower(0);
    }
}