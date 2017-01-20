package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Hit two beacons")
public class TwoBeacons extends OpModeBase {
    public void runOpMode() {
        super.runOpMode();
        telemetry.update();
        waitForStart();

        turnSpeed = .2;
        moveSpeed = .6;
        move(23, moveSpeed);

        if(allianceColor.equals("Red")) {
            turn(140, moveDirection.next(), turnSpeed);

            moveSpeed = .45;
            moveFast(-50, moveSpeed); //Movement to wall, no PID

            moveSpeed = .65;
            turn(20, moveDirection.next(), turnSpeed, 2); //Does not correct itself

            moveSpeed = .4;
            slowdownMin = .4;

            driveToWhiteLine(-moveSpeed, -moveSpeed);
            driveToWhiteLine(.1, .1);
            move(-9, moveSpeed);

            if (getColorName().equals(allianceColor)) {
                buttonPresser.setPosition(BUTTON_PRESSER_OUT);
                sleep(1000);
                move(8, moveSpeed); //Push past beacon with servo extended to hit beacon
            } else {
                move(9, moveSpeed); //Moves backwards to closer button
                if (getColorName().equals(allianceColor)) {
                    buttonPresser.setPosition(BUTTON_PRESSER_OUT);
                    sleep(1800);
                    move(9, moveSpeed); //Push past beacon with servo extended to hit beacon
                }
            }
            buttonPresser.setPosition(BUTTON_PRESSER_IN);

            //Second beacon
            move(12, moveSpeed); //Move past white line

            driveToWhiteLine(moveSpeed, moveSpeed); //Robot is facing away from start, move with positive power
            driveToWhiteLine(-.1, -.1);
            move(-9.5, moveSpeed); //Move to far button

            if (getColorName().equals(allianceColor)) {
                buttonPresser.setPosition(BUTTON_PRESSER_OUT);
                sleep(1800);
                move(8, moveSpeed); //Push past beacon with servo extended to hit beacon
            } else {
                move(10, moveSpeed); //Move to the close button
                if (getColorName().equals(allianceColor)) {
                    buttonPresser.setPosition(BUTTON_PRESSER_OUT);
                    sleep(1000);
                    move(9, moveSpeed); //Push past beacon with servo extended to hit beacon
                }
            }
        } else if(allianceColor.equals("Blue")) {
            turn(40, moveDirection, turnSpeed);

            moveSpeed = .45;
            moveFast(50, moveSpeed); //Movement to wall, no PID

            moveSpeed = .65;
            turn(20, moveDirection.next(), turnSpeed, 2); //Does not correct itself

            moveSpeed = .4;
            slowdownMin = .4;

            driveToWhiteLine(moveSpeed, moveSpeed);
            driveToWhiteLine(-.1, -.1); //Drive to exactly white line
            move(9, moveSpeed);

            if (getColorName().equals(allianceColor)) {
                buttonPresser.setPosition(BUTTON_PRESSER_OUT);
                sleep(1000);
                move(-8, moveSpeed); //Push past beacon with servo extended to hit beacon
            } else {
                move(-9, moveSpeed); //Moves backwards to closer button
                if (getColorName().equals(allianceColor)) {
                    buttonPresser.setPosition(BUTTON_PRESSER_OUT);
                    sleep(1800);
                    move(-9, moveSpeed); //Push past beacon with servo extended to hit beacon
                }
            }
            buttonPresser.setPosition(BUTTON_PRESSER_IN);

            //Second beacon
            move(-12, moveSpeed); //Move past white line

            driveToWhiteLine(-moveSpeed, -moveSpeed);
            driveToWhiteLine(.1, .1);
            move(10, moveSpeed); //Move to far button

            if (getColorName().equals(allianceColor)) {
                buttonPresser.setPosition(BUTTON_PRESSER_OUT);
                sleep(1800);
                move(-8, moveSpeed); //Push past beacon with servo extended to hit beacon
            } else {
                move(-10, moveSpeed); //Move to the close button
                if (getColorName().equals(allianceColor)) {
                    buttonPresser.setPosition(BUTTON_PRESSER_OUT);
                    sleep(1000);
                    move(-9, moveSpeed); //Push past beacon with servo extended to hit beacon
                }
            }
        }
        buttonPresser.setPosition(BUTTON_PRESSER_IN);
    }
}