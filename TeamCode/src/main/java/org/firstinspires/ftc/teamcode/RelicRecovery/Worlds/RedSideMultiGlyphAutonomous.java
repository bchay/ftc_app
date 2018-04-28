package org.firstinspires.ftc.teamcode.RelicRecovery.Worlds;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/*
This code is the code for the red side cryptobox, with multiple glyphs.
The robot hits the jewel, reads the VuMark, and drives off of the balancing stone.
 It then drives to the correct cryptobox column and deposits the jewel.
 */
@Autonomous(name = "Red Side Multi-Glyph Worlds", group = "Worlds")
public class RedSideMultiGlyphAutonomous extends OpModeBaseMultiGlyph {
    public void runOpMode() {
        super.runOpMode(OpModeType.AUTONOMOUS);

        hitJewel("Red");

        //Read VuMark to determine cryptobox keys
        RelicRecoveryVuMark vuMark = readVuMark("Red");

        if(vuMark.equals(RelicRecoveryVuMark.LEFT)) {
            move(14.5, Direction.FORWARD); //Drive off of balancing stone
            turn(70, Direction.LEFT);
            move(13, Direction.BACKWARD);
        } else if(vuMark.equals(RelicRecoveryVuMark.CENTER)) {
            move(13.5, Direction.FORWARD); //Drive off of balancing stone
            turn(63, Direction.LEFT);
            move(14, Direction.BACKWARD);
        } else if(vuMark.equals(RelicRecoveryVuMark.RIGHT)) {
            move(12, Direction.FORWARD); //Drive off of balancing stone
            turn(60, Direction.LEFT);
            move(14.5, Direction.BACKWARD); //Drive to cryptobox
        }

        flipGlyph(true, true);

        leftIntake.setPower(-.6); //Spin intake out to break up double glyph stacks
        rightIntake.setPower(-.6);

        if(!vuMark.equals(RelicRecoveryVuMark.RIGHT)) move(7, Direction.FORWARD);
        else move(9, Direction.FORWARD);

        leftIntake.setPower(.9); //Grab glyphs
        rightIntake.setPower(.9);
        move(2, Direction.FORWARD);

        sleep(2000);

        leftIntake.setPower(0);
        rightIntake.setPower(0);

        if(!vuMark.equals(RelicRecoveryVuMark.RIGHT)) turn(3, Direction.LEFT, turnSpeed, 0, 1000);
        else turn(2, Direction.LEFT, turnSpeed, 0, 1000);

        glyphLever.setPosition(GLYPH_LEVER_UP);
        sleep(600);
        glyphLever.setPosition(GLYPH_LEVER_DOWN);
        sleep(200);

        leftIntake.setPower(-1);
        rightIntake.setPower(-1);

        new Thread(new Runnable() {
            @Override
            public void run() {
                sleep(800);
                leftIntake.setPower(0);
                rightIntake.setPower(0);
            }
        }).start();

        if(vuMark.equals(RelicRecoveryVuMark.LEFT)) {
            new Thread(new Runnable() {
                @Override
                public void run() {
                    sleep(1300);
                    glyphLift.setPower(1);

                    sleep(1600);
                    glyphLift.setPower(0);
                }
            }).start();
        }

        if(vuMark.equals(RelicRecoveryVuMark.LEFT)) {
            move(13, Direction.BACKWARD, .8, false, 10000);
        } else if(vuMark.equals(RelicRecoveryVuMark.CENTER)) {
            turn(2, Direction.LEFT, turnSpeed, 0, 5000);
            move(13, Direction.BACKWARD, .8, false, 10000);
        }  else if(vuMark.equals(RelicRecoveryVuMark.RIGHT)) {
            move(11, Direction.BACKWARD, .8, false, 10000);
        }

        flipGlyph(false, false);

        moveSpeedMax += .1;
        if(vuMark.equals(RelicRecoveryVuMark.RIGHT)) {
            move(6, Direction.BACKWARD); //Push glyph into cryptobox
            move(4, Direction.FORWARD); //Move away from cryptobox to avoid hitting glyphs
        } else {
            move(5, Direction.BACKWARD); //Push glyph into cryptobox
            move(3, Direction.FORWARD); //Move away from cryptobox to avoid hitting glyphs
        }
    }
}
