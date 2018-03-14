package org.firstinspires.ftc.teamcode.RelicRecovery.State;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name = "Blue Center State - Multiple Glyphs", group = "State")
public class BlueCenterMultipleGlyphsAutonomous extends OpModeBase {
    public void runOpMode() {
        super.runOpMode(OpModeType.AUTONOMOUS);
        hitJewelFast("Blue");

        //Read VuMark to determine cryptobox key
        RelicRecoveryVuMark vuMark = readVuMark("Blue");

        turnSpeed = .95;
        moveSpeedMax = 1;
        turnSpeedMin = .45;

        //Autonomous movement code
        if(vuMark.equals(RelicRecoveryVuMark.RIGHT)) {
            move(28, Direction.BACKWARD);
            turn(55, Direction.LEFT); //Turn so that back of robot is facing cryptobox

            move(7.5, Direction.BACKWARD); //Move towards cryptobox

            //Deposit glyph
            glyphStopper.setPosition(GLYPH_STOPPER_UP);
            sleep(500);
            glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
            sleep(900);

            move(7, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up

            //Multiple Glyphs
            glyphFlipper.setPosition(GLYPH_FLIPPER_FLAT);
            glyphStopper.setPosition(GLYPH_STOPPER_DOWN);
            turn(45, Direction.LEFT);



            moveIntake.setPower(-1); //Move intake down
            sleep(2000);
            moveIntake.setPower(0);

            leftIntake.setPower(1); //Suck in
            rightIntake.setPower(-1);

            move(19, Direction.FORWARD); //Drive to glyph pit
            sleep(800);

            new Thread(new Runnable() {
                @Override
                public void run() {
                    sleep(300);
                    glyphFlipper.setPosition(GLYPH_FLIPPER_PARTIALLY_UP - .05);
                    sleep(300);
                    glyphFlipper.setPosition(GLYPH_FLIPPER_FLAT);
                    sleep(300);

                    glyphFlipper.setPosition(GLYPH_FLIPPER_PARTIALLY_UP - .05);
                    sleep(300);
                    glyphFlipper.setPosition(GLYPH_FLIPPER_FLAT);
                    sleep(300);
                }
            }).start();

            new Thread(new Runnable() {
                @Override
                public void run() {
                    leftIntake.setPower(1);
                    rightIntake.setPower(-.5);
                    sleep(700);

                    leftIntake.setPower(.5);
                    rightIntake.setPower(-1);
                    sleep(700);
                }
            }).start();

            new Thread(new Runnable() {
                @Override
                public void run() {
                    sleep(700); //Dump glyphs while moving forward to cryptobox
                    glyphStopper.setPosition(GLYPH_STOPPER_UP); //Dump any extra glyphs
                    sleep(500);
                    glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
                    sleep(700);
                    glyphLever.setPosition(GLYPH_LEVER_DOWN_FLIPPER);
                }
            }).start();

            new Thread(new Runnable() {
                @Override
                public void run() {
                    sleep(300);
                    glyphLift.setPower(-1);
                    sleep(2000);
                    glyphLift.setPower(0);
                }
            }).start();

            move(18.5, Direction.BACKWARD); //Drive to cryptobox

            leftIntake.setPower(-1); //Eject glyphs
            rightIntake.setPower(1);
            sleep(200);

            move(4, Direction.FORWARD); //Back up
            move(10, Direction.BACKWARD); //Drive to cryptobox

            move(5, Direction.FORWARD); //Back away from cryptobox so that the robot is not touching the glyphs

            leftIntake.setPower(0);
            rightIntake.setPower(0);
        } else if(vuMark.equals(RelicRecoveryVuMark.CENTER)) {
            move(45.5, Direction.BACKWARD);
            turn(125, Direction.LEFT); //Turn so that back of robot is facing cryptobox

            move(5.5, Direction.BACKWARD); //Move towards cryptobox

            //Deposit glyph
            glyphStopper.setPosition(GLYPH_STOPPER_UP);
            sleep(500);
            glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
            sleep(1000);

            move(8, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up


            //Multiple Glyphs
            glyphFlipper.setPosition(GLYPH_FLIPPER_FLAT);
            glyphStopper.setPosition(GLYPH_STOPPER_DOWN);

            turn(20, Direction.RIGHT);
            move(10, Direction.RIGHT);



            moveIntake.setPower(-1); //Move intake down
            sleep(2000);
            moveIntake.setPower(0);

            leftIntake.setPower(1); //Suck in
            rightIntake.setPower(-1);

            move(17, Direction.FORWARD); //Drive to glyph pit
            sleep(800);

            new Thread(new Runnable() {
                @Override
                public void run() {
                    glyphFlipper.setPosition(GLYPH_FLIPPER_PARTIALLY_UP - .07);
                    sleep(300);
                    glyphFlipper.setPosition(GLYPH_FLIPPER_FLAT);
                    sleep(300);

                    glyphFlipper.setPosition(GLYPH_FLIPPER_PARTIALLY_UP - .07);
                    sleep(300);
                    glyphFlipper.setPosition(GLYPH_FLIPPER_FLAT);
                    sleep(300);
                }
            }).start();

            new Thread(new Runnable() {
                @Override
                public void run() {
                    leftIntake.setPower(1);
                    rightIntake.setPower(-.5);
                    sleep(700);

                    leftIntake.setPower(.5);
                    rightIntake.setPower(-1);
                    sleep(700);
                }
            }).start();

            new Thread(new Runnable() {
                @Override
                public void run() {
                    sleep(500); //Dump glyphs while moving forward to cryptobox
                    glyphStopper.setPosition(GLYPH_STOPPER_UP); //Dump any extra glyphs
                    sleep(500);
                    glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
                    sleep(700);
                    glyphLever.setPosition(GLYPH_LEVER_DOWN_FLIPPER);
                }
            }).start();

            new Thread(new Runnable() {
                @Override
                public void run() {
                    glyphLift.setPower(-1);
                    sleep(2000);
                    glyphLift.setPower(0);
                }
            }).start();

            move(16, Direction.BACKWARD); //Drive to cryptobox

            leftIntake.setPower(-1); //Eject glyphs
            rightIntake.setPower(1);
            sleep(500);

            move(4, Direction.FORWARD); //Back up
            move(10, Direction.BACKWARD); //Drive to cryptobox

            move(5, Direction.FORWARD); //Back away from cryptobox so that the robot is not touching the glyphs

            leftIntake.setPower(0);
            rightIntake.setPower(0);
        } else if(vuMark.equals(RelicRecoveryVuMark.LEFT)) {
            move(38, Direction.BACKWARD);
            turn(125, Direction.LEFT); //Turn so that back of robot is facing cryptobox

            move(5.5, Direction.BACKWARD); //Move towards cryptobox

            //Deposit glyph
            glyphStopper.setPosition(GLYPH_STOPPER_UP);
            sleep(500);
            glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
            sleep(1000);

            move(8, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up


            //Multiple Glyphs
            glyphFlipper.setPosition(GLYPH_FLIPPER_FLAT);
            glyphStopper.setPosition(GLYPH_STOPPER_DOWN);
            turn(5, Direction.RIGHT);


            moveIntake.setPower(-1); //Move intake down
            sleep(2000);
            moveIntake.setPower(0);

            leftIntake.setPower(1); //Suck in
            rightIntake.setPower(-1);

            move(17, Direction.FORWARD); //Drive to glyph pit
            sleep(800);

            new Thread(new Runnable() {
                @Override
                public void run() {
                    glyphFlipper.setPosition(GLYPH_FLIPPER_PARTIALLY_UP - .07);
                    sleep(300);
                    glyphFlipper.setPosition(GLYPH_FLIPPER_FLAT);
                    sleep(300);

                    glyphFlipper.setPosition(GLYPH_FLIPPER_PARTIALLY_UP - .07);
                    sleep(300);
                    glyphFlipper.setPosition(GLYPH_FLIPPER_FLAT);
                    sleep(300);
                }
            }).start();

            new Thread(new Runnable() {
                @Override
                public void run() {
                    leftIntake.setPower(1);
                    rightIntake.setPower(-.5);
                    sleep(700);

                    leftIntake.setPower(.5);
                    rightIntake.setPower(-1);
                    sleep(700);
                }
            }).start();

            new Thread(new Runnable() {
                @Override
                public void run() {
                    sleep(600); //Dump glyphs while moving forward to cryptobox
                    glyphStopper.setPosition(GLYPH_STOPPER_UP); //Dump any extra glyphs
                    sleep(500);
                    glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
                    sleep(700);
                    glyphLever.setPosition(GLYPH_LEVER_DOWN_FLIPPER);
                }
            }).start();

            new Thread(new Runnable() {
                @Override
                public void run() {
                    glyphLift.setPower(-1);
                    sleep(2000);
                    glyphLift.setPower(0);
                }
            }).start();

            move(20, Direction.BACKWARD); //Drive to cryptobox

            leftIntake.setPower(-1); //Eject glyphs
            rightIntake.setPower(1);
            sleep(500);

            move(4, Direction.FORWARD); //Back up
            move(10, Direction.BACKWARD); //Drive to cryptobox

            move(5, Direction.FORWARD); //Back away from cryptobox so that the robot is not touching the glyphs

            leftIntake.setPower(0);
            rightIntake.setPower(0);
        }
    }
}
