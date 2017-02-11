package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * The robot starts at the inside right side of the third tile in the direction of the center vortex.
 * The robot is placed perpendicular to the wall, facing the beacon.
 * Two particles are loaded, one into the sweeper and one into the shooting chamber.
 * * This OpMode shoots two particles into the Center Vortex, hits the Cap Ball, and presses both beacons.
 */
@Autonomous(name = "Two balls, two beacons")
public class WallRiderAutonomous extends OpModeBase {
    public void runOpMode() {
        super.runOpMode();
        waitForStart();

        //Beginning of actual code
        move(20, moveSpeed, true, kP, 1.3, .05);
        shoot();

        if(allianceColor.equals("Red")) {
            turn(45, moveDirection, .2); //Initial turn towards wall
            move(64, .9, true, .025, 1.3, .05);
            turn(165, Direction.LEFT, .4, 0); //Turn backwards to use rider wheels
            move(-14, moveSpeed, false, 0, 0, .2);
            turn(13, Direction.RIGHT, turnSpeed, 0);
            move(-13.5, moveSpeed, false, 0, 0, .2);

            driveToWhiteLine(.13, .11); //Drive forward, robot is reversed
            move(-1, moveSpeed, false, 0, 0, .2); //0 is no speed reduction

            //Robot is positioned with button presser next to far button of far beacon
            if (getColorName().equals(allianceColor)) {
                move(-1.5, .4, false, 0, 0, 0);
                buttonPresser.setPosition(BUTTON_PRESSER_OUT);
                sleep(1100);
                //move(-3, .2, false, 0, 0, .2);
                buttonPresser.setPosition(BUTTON_PRESSER_IN);
                sleep(1100);
                //move(3, .4, false);
            } else {
                move(2, moveSpeed); //Moves to closer button
                if (getColorName().equals(allianceColor)) {
                    buttonPresser.setPosition(BUTTON_PRESSER_OUT);
                    sleep(1100);
                    //move(-2, moveSpeed, false, 0, 0, .2); //0 is no speed reduction
                }
                buttonPresser.setPosition(BUTTON_PRESSER_IN);
            }

            //Second beacon
            move(26, .75, false, kP, 1.3, .05);

            driveToWhiteLine(.2, .2); //Drive forward, robot is reversed
            move(-2, moveSpeed, false, 0, 0, .2); //0 is no speed reduction

            if (getColorName().equals(allianceColor)) {
                move(-1.2, moveSpeed, false, 0, 0, .2); //0 is no speed reduction
                buttonPresser.setPosition(BUTTON_PRESSER_OUT);
                sleep(1100);
                //move(-2, .2, false, 0, 0, .2);
                buttonPresser.setPosition(BUTTON_PRESSER_IN);
                sleep(900);
                //move(3, .4); //Move to hit beacon (button presser is angled)
            } else {
                move(2, moveSpeed); //Moves to closer button
                if (getColorName().equals(allianceColor)) {
                    buttonPresser.setPosition(BUTTON_PRESSER_OUT);
                    sleep(900);
                    //move(-3, moveSpeed, false, 0, 0, .2); //Move to hit beacon (button presser is angled)
                }
            }
            buttonPresser.setPosition(BUTTON_PRESSER_IN);
            sleep(500);
        } else if(allianceColor.equals("Blue")) {
            turn(45, moveDirection, .2); //Initial turn towards wall
            move(63, .9, false, .025, 1.3, .05);

            turn(23, Direction.LEFT, .4, 0);
            move(13, moveSpeed, false, 0, 0, .2);
            turn(15, Direction.LEFT, turnSpeed, 0);
            move(29, moveSpeed, false, 0, 0, .2); //Move to far beacon

            driveToWhiteLine(-.16, -.11);
            move(2, moveSpeed, false, 0, 0, .2);

            //Robot is positioned with button presser next to far button of far beacon
            if (getColorName().equals(allianceColor)) {
                //move(1, .4, false, 0, 0, 0);
                buttonPresser.setPosition(BUTTON_PRESSER_OUT);
                sleep(1100);
                //move(3, .2, false, 0, 0, .2);
                buttonPresser.setPosition(BUTTON_PRESSER_IN);
                sleep(1100);
                //move(3, .4, false);
            } else { //Closer button is alliance color
                move(-2.5, moveSpeed); //Moves to closer button
                if (getColorName().equals(allianceColor)) {
                    buttonPresser.setPosition(BUTTON_PRESSER_OUT);
                    sleep(1100);
                    //move(-2, moveSpeed, false, 0, 0, .2); //0 is no speed reduction
                }
                buttonPresser.setPosition(BUTTON_PRESSER_IN);
            }

            //Second beacon
            move(-14, .75, false, kP, 1.3, .05);
            turn(6, Direction.LEFT); //Orient towards wall
            move(-13, .75, false, kP, 1.3, .05);

            driveToWhiteLine(-.23, -.2);
            move(3.5, moveSpeed, false, 0, 0, .2);

            if (getColorName().equals(allianceColor)) { //Positioned at far button of first beacon
                //move(-1, moveSpeed, false, 0, 0, .2);
                buttonPresser.setPosition(BUTTON_PRESSER_OUT);
                sleep(1100);
                //move(-3, .4); //Move to hit beacon
            } else {
                if (getColorName().equals(allianceColor)) {
                    buttonPresser.setPosition(BUTTON_PRESSER_OUT);
                    sleep(900);
                    //move(-3, moveSpeed, false, 0, 0, .2); //Move to hit beacon (button presser is angled)
                }
            }
            buttonPresser.setPosition(BUTTON_PRESSER_IN);
            sleep(500);
        }
    }
}