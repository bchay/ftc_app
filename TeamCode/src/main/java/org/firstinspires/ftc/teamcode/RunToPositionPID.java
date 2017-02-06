package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "Run to Position PID", group = "Test Code")
public class RunToPositionPID extends OpModeBase {
    private int distance = 2000;
    private double speed = .6;
    private double kP = .05;

    public void runOpMode() {
        super.runOpMode();
        waitForStart();

        int initialHeading = gyro.getIntegratedZValue();

        motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLeftFront.setTargetPosition(distance);
        motorLeftBack.setTargetPosition(distance);
        motorRightFront.setTargetPosition(distance);
        motorRightBack.setTargetPosition(distance);

        motorLeftFront.setPower(speed);
        motorLeftBack.setPower(speed);
        motorRightFront.setPower(speed);
        motorRightBack.setPower(speed);

        while (motorLeftFront.isBusy() && motorLeftBack.isBusy() && motorRightFront.isBusy() && motorRightBack.isBusy() && opModeIsActive()) {
            //Only one encoder target must be reached
            double error = initialHeading - gyro.getIntegratedZValue();
            double targetError = error * kP;

            motorLeftFront.setPower(Range.clip(speed - targetError, 0, 1));
            motorLeftBack.setPower(Range.clip(speed - targetError, 0, 1));
            motorRightFront.setPower(Range.clip(speed + targetError, 0, 1));
            motorRightBack.setPower(Range.clip(speed + targetError, 0, 1));

            telemetry.addData("Left motor power", motorLeftFront.getPower());
            telemetry.addData("Right motor power", motorRightFront.getPower());
            telemetry.addData("Current Heading", gyro.getIntegratedZValue());
            telemetry.addData("Error", targetError);
            telemetry.update();
        }

        motorLeftFront.setPower(0);
        motorLeftBack.setPower(0);
        motorRightFront.setPower(0);
        motorRightBack.setPower(0);
        sleep(5000);
    }
}
