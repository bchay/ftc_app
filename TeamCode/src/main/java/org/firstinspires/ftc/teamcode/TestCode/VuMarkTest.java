package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.VuMarkReader;

@TeleOp(name = "Read VuMark", group = "Test Code")
public class VuMarkTest extends LinearOpMode {
    public void runOpMode() {
        VuMarkReader vuMarkReader = new VuMarkReader(hardwareMap);
        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("VuMark", vuMarkReader.getVuMark());
            telemetry.update();
        }
    }
}
