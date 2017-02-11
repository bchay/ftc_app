package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Shoot two, cap ball")
public class ShootTwoCapBall extends OpModeBase {
    public void runOpMode() {
        super.runOpMode();
        waitForStart();

        move(33, moveSpeed, true, kP, 1.3, .05);
        shoot(10000); //Shoot with 10 second timeout
        sleep(10000);
        move(25);
    }
}