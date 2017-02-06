package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "kP Calculator", group = "Configuration")
public class KPCalculator extends OpModeBase {
    public void runOpMode() {
        super.runOpMode();
        double previousTime = System.currentTimeMillis();

        while(!isStopRequested() && !opModeIsActive()) {
            if(gamepad1.dpad_up && System.currentTimeMillis() - previousTime > 200) {
                kP += .001;
                previousTime = System.currentTimeMillis();
            } else if(gamepad1.dpad_down && System.currentTimeMillis() - previousTime > 200) {
                kP -= .001;
                previousTime = System.currentTimeMillis();
            }

            telemetry.addData("kP", kP);
            telemetry.update();

            sleep(50);
            idle();
        }
        if(opModeIsActive()) move(48, moveSpeed);
    }
}
