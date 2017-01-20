package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//Robot begins at fifth tile furthest from corner vortex, 45 deg angle facing center vortex
@Autonomous(name = "Angle and cap ball")
public class ShootTwoAngleCapBall extends OpModeBase {
    public void runOpMode() {
        super.runOpMode();
        moveSpeed -= .5;
        waitForStart();

        move(42, moveSpeed);
        shoot();
        move(34, moveSpeed);
    }

    public void shoot() {
        shooter.setPower(1);
        sleep(1200);
        shooter.setPower(0); //First ball fired

        intake.setPower(1);
        sleep(500);
        ballStop.setPosition(BALL_STOP_UP);
        sleep(1000);
        intake.setPower(0); //Second ball loaded

        shooter.setPower(1);
        sleep(1300);
        shooter.setPower(0);
    }
}