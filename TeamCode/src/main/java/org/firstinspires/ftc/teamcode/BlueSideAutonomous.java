package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

public class BlueSideAutonomous extends OpModeBase {
    public void runOpMode() {
        super.runOpMode(OpModeType.AUTONOMOUS);
        hitJewel("Blue");

        //Read VuMark to determine cryptobox key
        RelicRecoveryVuMark vuMark = readVuMark("Blue");


        //Autonomous movement code
        move(31, Direction.BACKWARD);

        if(vuMark.equals(RelicRecoveryVuMark.RIGHT)) {
            move(17, Direction.LEFT); //Strafe toward right column

            turn(23, Direction.RIGHT);
            move(2, Direction.BACKWARD, moveSpeedMax, false, 1000); //Move toward cryptobox

            //Deposit glyph
            glyphStopper.setPosition(GLYPH_STOPPER_UP);
            sleep(500);
            glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
            sleep(1000);

            move(4, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up
        } else if(vuMark.equals(RelicRecoveryVuMark.CENTER)) {
            move(7.5, Direction.LEFT);

            turn(25, Direction.RIGHT);
            move(1.5, Direction.BACKWARD, moveSpeedMax, false, 1000); //Move toward cryptobox

            //Deposit glyph
            glyphStopper.setPosition(GLYPH_STOPPER_UP);
            sleep(500);
            glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
            sleep(1000);

            move(4, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up
        } else if(vuMark.equals(RelicRecoveryVuMark.LEFT)) {
            move(2.5, Direction.LEFT);

            turn(19, Direction.RIGHT);
            move(1, Direction.FORWARD); //Back up before dumping glyph

            //Deposit glyph
            glyphStopper.setPosition(GLYPH_STOPPER_UP);
            sleep(500);
            glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
            sleep(1000);

            move(5, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up
        }
    }
}
