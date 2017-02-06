package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
            turn(10, Direction.RIGHT, turnSpeed, 0);
            move(-13.5, moveSpeed, false, 0, 0, .2);

            driveToWhiteLine(.13, .11); //Drive forward, robot is reversed
            move(-2, moveSpeed, false, 0, 0, .2); //0 is no speed reduction

            if (getColorName().equals(allianceColor)) {
                buttonPresser.setPosition(BUTTON_PRESSER_OUT);
                sleep(900);
                move(-2, .2, false, 0, 0, .2);
                buttonPresser.setPosition(BUTTON_PRESSER_IN);
                sleep(1000);
                move(3, .4, false);
            } else {
                move(3, moveSpeed); //Moves to closer button
                if (getColorName().equals(allianceColor)) {
                    buttonPresser.setPosition(BUTTON_PRESSER_OUT);
                    sleep(900);
                    move(-2, moveSpeed, false, 0, 0, .2); //0 is no speed reduction
                }
                buttonPresser.setPosition(BUTTON_PRESSER_IN);
            }

            //Second beacon
            move(26, .75, true, kP, 1.3, .05);

            driveToWhiteLine(.16, .14); //Drive forward, robot is reversed
            move(-2, moveSpeed, false, 0, 0, .2); //0 is no speed reduction

            if (getColorName().equals(allianceColor)) {
                buttonPresser.setPosition(BUTTON_PRESSER_OUT);
                sleep(900);
                move(-2, .2, false, 0, 0, .2);
                buttonPresser.setPosition(BUTTON_PRESSER_IN);
                sleep(900);
                move(3, .4); //Move to hit beacon (button presser is angled)
            } else {
                move(3, moveSpeed); //Moves to closer button
                if (getColorName().equals(allianceColor)) {
                    buttonPresser.setPosition(BUTTON_PRESSER_OUT);
                    sleep(900);
                    move(-3, moveSpeed, false, 0, 0, .2); //Move to hit beacon (button presser is angled)
                }
            }
            buttonPresser.setPosition(BUTTON_PRESSER_IN);
            sleep(500);
        } else if(allianceColor.equals("Blue")) {
            turn(45, moveDirection, .2); //Initial turn towards wall
            move(64, .9, true, .025, 1.3, .05);
            turn(28, Direction.RIGHT, .4); //Turn towards wall
            move(13.5, moveSpeed, false, 0, 0, .2);
            turn(10, Direction.LEFT, turnSpeed, 0);
            move(13.5, moveSpeed, false, 0, 0, .2);

            driveToWhiteLine(-.13, -.11); //Drive forward, robot is reversed
            move(2, moveSpeed, false, 0, 0, .2); //0 is no speed reduction
/*
            if (getColorName().equals(allianceColor)) {
                buttonPresser.setPosition(BUTTON_PRESSER_OUT);
                sleep(900);
                move(-2, .2, false, 0, 0, .2);
                buttonPresser.setPosition(BUTTON_PRESSER_IN);
                sleep(1000);
                move(6, .4);
            } else {
                move(3, moveSpeed); //Moves to closer button
                if (getColorName().equals(allianceColor)) {
                    buttonPresser.setPosition(BUTTON_PRESSER_OUT);
                    sleep(900);
                    move(-2, moveSpeed, false, 0, 0, .2); //0 is no speed reduction
                }
                buttonPresser.setPosition(BUTTON_PRESSER_IN);
            }

            //Second beacon
            move(26, .75, true, kP, 1.3, .05);

            driveToWhiteLine(.16, .14); //Drive forward, robot is reversed
            move(-2, moveSpeed, false, 0, 0, .2); //0 is no speed reduction

            if (getColorName().equals(allianceColor)) {
                buttonPresser.setPosition(BUTTON_PRESSER_OUT);
                sleep(900);
                move(-2, .2, false, 0, 0, .2);
                buttonPresser.setPosition(BUTTON_PRESSER_IN);
                sleep(900);
                move(3, .4);
            } else {
                move(3, moveSpeed); //Moves to closer button
                if (getColorName().equals(allianceColor)) {
                    buttonPresser.setPosition(BUTTON_PRESSER_OUT);
                    sleep(900);
                    move(-2, moveSpeed, false, 0, 0, .2); //0 is no speed reduction
                }
            }
            buttonPresser.setPosition(BUTTON_PRESSER_IN);
            sleep(500);
        }
        buttonPresser.setPosition(BUTTON_PRESSER_IN);
        */
        }
    }
}