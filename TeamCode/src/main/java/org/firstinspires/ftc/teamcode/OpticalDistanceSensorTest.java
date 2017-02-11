package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

@TeleOp(name = "ODS Test", group = "Test Code")
public class OpticalDistanceSensorTest extends OpModeBase {
    public void runOpMode() {
        super.runOpMode();
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("Line Raw", odsLine.getRawLightDetected());
            telemetry.addData("Line Light", odsLine.getLightDetected());
            telemetry.addData("Ball Raw", odsBall.getRawLightDetected());
            telemetry.addData("Ball Light", odsBall.getLightDetected());
            telemetry.addData("Ball Detected", odsBallDetected());
            telemetry.update();
        }
    }
}