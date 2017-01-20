package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Two balls, two beacons")
public class WallRiderAutonomous extends OpModeBase {
    public void runOpMode() {
        super.runOpMode();
        telemetry.update();
        waitForStart();

        turnSpeed = .2;
        moveSpeed = .6;
        move(23, moveSpeed);
        shoot();

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
            turn(70, moveDirection, turnSpeed); //Turn towards beacon

            moveSpeed = .45;
            moveFast(78, moveSpeed); //Movement to wall, no PID

            moveSpeed = .65;
            turn(25, moveDirection.next(), turnSpeed, 2); //Does not correct itself

            moveSpeed = .4;
            slowdownMin = .4;

            driveToWhiteLine(moveSpeed, moveSpeed);
            driveToWhiteLine(-.2, -.2); //Drive to exactly white line

            move(7, moveSpeed);

            if (getColorName().equals(allianceColor) || true) { //Furthest button on second beacon is alliance color
                buttonPresser.setPosition(BUTTON_PRESSER_OUT);
                sleep(1000);
                move(8, moveSpeed); //Push past beacon with servo extended to hit beacon
            } else {
                move(-9, moveSpeed); //Moves backwards to closer button
                if (getColorName().equals(allianceColor)) {
                    buttonPresser.setPosition(BUTTON_PRESSER_OUT);
                    sleep(1800);
                    move(9, moveSpeed); //Push past beacon with servo extended to hit beacon
                }
            }
            buttonPresser.setPosition(BUTTON_PRESSER_IN);

            //Second beacon
            move(-14, moveSpeed); //Move past white line

            driveToWhiteLine(-moveSpeed, -moveSpeed);
            driveToWhiteLine(.1, .1);
            move(10, moveSpeed); //Move to far button

            if (getColorName().equals(allianceColor) || true) { //Furthest button on close beacon is alliance color
                buttonPresser.setPosition(BUTTON_PRESSER_OUT);
                sleep(1800);
                move(8, moveSpeed); //Push past beacon with servo extended to hit beacon
            } else { //Closest button on close beacon is alliance color
                move(-10, moveSpeed); //Move to the close button
                if (getColorName().equals(allianceColor)) {
                    buttonPresser.setPosition(BUTTON_PRESSER_OUT);
                    sleep(1000);
                    move(9, moveSpeed); //Push past beacon with servo extended to hit beacon
                }
            }
        }
        buttonPresser.setPosition(BUTTON_PRESSER_IN);
    }
}