package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//Robot begins at fifth tile furthest from corner vortex, 45 deg angle facing center vortex
@Autonomous(name = "Shoot two at angle")
public class ShootTwoAngle extends OpModeBase {
    public void runOpMode() {
        super.runOpMode();
        moveSpeed -= .5;

        waitForStart();

        move(42, moveSpeed);
        shoot();
    }

    public void shoot() {
        //Shoot balls
        shooter.setPower(1);
        intake.setPower(1);
        sleep(300);
        ballStop.setPosition(BALL_STOP_UP);
        sleep(900);
        shooter.setPower(0);

        //Second ball
        sleep(1800); //Intake still running
        intake.setPower(0);
        shooter.setPower(1);
        sleep(1000);
        shooter.setPower(0);
    }
}