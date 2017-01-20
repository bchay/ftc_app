package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Shoot two balls")
public class ShootTwo extends OpModeBase {
    public void runOpMode() {
        super.runOpMode();

        moveSpeed -= .2;
        waitForStart();

        move(23, moveSpeed);
        shoot();
    }
}