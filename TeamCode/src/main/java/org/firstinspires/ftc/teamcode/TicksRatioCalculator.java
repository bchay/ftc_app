package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Ticks Ratio Calculator", group = "Configuration")
public class TicksRatioCalculator extends OpModeBase {
    private double ticks = 0;

    public void runOpMode() {
        super.runOpMode();
        ticksRatio = 1;
        double previousTime = System.currentTimeMillis();

        while(!isStopRequested() && !opModeIsActive()) {
            if(gamepad1.dpad_up && System.currentTimeMillis() - previousTime > 500) {
                ticks += 500;
                previousTime = System.currentTimeMillis();
            } else if(gamepad1.dpad_down && System.currentTimeMillis() - previousTime > 500) {
                ticks -= 500;
                previousTime = System.currentTimeMillis();
            }

            telemetry.addData("Ticks", ticks);
            telemetry.update();

            sleep(50);
            idle();
        }
        if(opModeIsActive()) move(ticks, moveSpeed);
    }
}
