package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Lift to Position Test", group = "Test Code")
public class LiftToPosition extends LinearOpMode {
    DcMotor lift;

    public void runOpMode() {
        lift = hardwareMap.dcMotor.get("lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        lift.setTargetPosition(1000);
        lift.setPower(1);

        while(lift.isBusy()) {
            telemetry.addData("Lift Current Position", lift.getCurrentPosition());
            telemetry.addData("Lift Target Position", lift.getTargetPosition());
            telemetry.update();
        }
        lift.setPower(0);
    }
}
