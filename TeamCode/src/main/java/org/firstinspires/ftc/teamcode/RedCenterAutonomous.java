package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

public class RedCenterAutonomous extends OpModeBase {
    public void runOpMode() {
        super.runOpMode(OpModeType.AUTONOMOUS);
        hitJewel("Red");

        //Read VuMark to determine cryptobox key
        RelicRecoveryVuMark vuMark = readVuMark("Red");

        if(vuMark.equals(RelicRecoveryVuMark.RIGHT)) {
            move(25, Direction.FORWARD);

            turn(105, Direction.LEFT); //Turn so that back of robot is facing cryptobox
            move(2, Direction.BACKWARD, moveSpeedMax, false, 1000); //Move away from cryptobox


            //Deposit glyph
            glyphStopper.setPosition(GLYPH_STOPPER_UP);
            sleep(500);
            glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
            sleep(1000);

            move(4, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up
        } else if(vuMark.equals(RelicRecoveryVuMark.CENTER)) {
            move(29, Direction.FORWARD);

            turn(120, Direction.LEFT, .8); //Turn so that back of robot is facing cryptobox
            move(3.5, Direction.BACKWARD, moveSpeedMax, false, 1000); //Move towards cryptobox

            //Deposit glyph
            glyphStopper.setPosition(GLYPH_STOPPER_UP);
            sleep(400);
            glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
            sleep(600);

            move(4.5, Direction.FORWARD, moveSpeedMax, false, 1000); //Move away from cryptobox
        } else if(vuMark.equals(RelicRecoveryVuMark.LEFT)) {
            sleep(1000);
            move(34.5, Direction.FORWARD);

            turn(125, Direction.LEFT); //Turn so that back of robot is facing cryptobox
            move(5, Direction.BACKWARD, moveSpeedMax, false, 1000); //Move closer to cryptobox

            //Deposit glyph
            glyphStopper.setPosition(GLYPH_STOPPER_UP);
            sleep(500);
            glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
            sleep(1000);

            move(4, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up
        }
    }
}
