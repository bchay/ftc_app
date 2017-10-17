package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.VuMarkReader;

@Autonomous(name = "Read VuMark")
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
