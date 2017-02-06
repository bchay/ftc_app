package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Ramp Down Calculator", group = "Configuration")
public class RampDownCalculator extends OpModeBase {
    private double a;
    private double b;

    public void runOpMode() {
        super.runOpMode();
        double previousTime = System.currentTimeMillis();

        while(!isStopRequested() && !opModeIsActive()) {
            if(gamepad1.dpad_up && System.currentTimeMillis() - previousTime > 200) {
                a += .01;
                previousTime = System.currentTimeMillis();
            } else if(gamepad1.dpad_down && System.currentTimeMillis() - previousTime > 200) {
                a -= .01;
                previousTime = System.currentTimeMillis();
            } else if(gamepad1.y && System.currentTimeMillis() - previousTime > 200) {
                b += .01;
                previousTime = System.currentTimeMillis();
            } else if(gamepad1.a && System.currentTimeMillis() - previousTime > 200) {
                b -= .01;
                previousTime = System.currentTimeMillis();
            }

            telemetry.addData("a", a);
            telemetry.addData("b", b);
            telemetry.update();

            sleep(50);
            idle();
        }
        if(opModeIsActive()) move(2000, moveSpeed, true, kP, a, b);
    }
}
