package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * The robot starts at the at a 45 degree angle facing the center vortex.
 * The robot is positioned on the side opposite to the corner vortex.
 * Two particles are loaded, one into the sweeper and one into the shooting chamber.
 * This OpMode shoots two particles into the Center Vortex, knocks the alliance Cap Ball, and parks on the Center Vortex.
 */
@Autonomous(name = "Shoot two, cap ball")
public class ShootTwoCapBall extends OpModeBase {
    public void runOpMode() {
        super.runOpMode();
        waitForStart();

        move(33, moveSpeed, true, kP, 1.3, .05);
        shoot(10000); //Shoot with 10 second timeout
        sleep(10000); //Wait 10 seconds
        move(25);
    }
}