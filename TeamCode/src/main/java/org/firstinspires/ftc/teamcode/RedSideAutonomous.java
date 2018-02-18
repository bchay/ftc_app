package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name = "Red Side", group = "Autonomous")
public class RedSideAutonomous extends OpModeBase {
    public void runOpMode() {
        super.runOpMode(OpModeType.AUTONOMOUS);
        hitJewel("Red");

        //Read VuMark to determine cryptobox key
        RelicRecoveryVuMark vuMark = readVuMark("Red");


        //Autonomous movement code
        if(vuMark.equals(RelicRecoveryVuMark.LEFT)) {
            move(28, Direction.FORWARD);
            move(15.5, Direction.LEFT); //Strafe to left column

            turn(150, Direction.RIGHT, .95, 2, 10000); //Reverse robot

            move(2, Direction.BACKWARD);

            //Deposit glyph
            glyphStopper.setPosition(GLYPH_STOPPER_UP);
            sleep(500);
            glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
            sleep(1000);

            move(5, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up
        } else if(vuMark.equals(RelicRecoveryVuMark.CENTER)) {
            move(28, Direction.FORWARD);
            move(9, Direction.LEFT); //Strafe to align with column
            turn(153, Direction.RIGHT, .95); //Reverse robot

            //Deposit glyph
            glyphStopper.setPosition(GLYPH_STOPPER_UP);
            sleep(500);
            glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
            sleep(1000);

            move(5, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up
        } else if(vuMark.equals(RelicRecoveryVuMark.RIGHT)) {
            move(28, Direction.FORWARD);
            turn(150, Direction.RIGHT); //Reverse robot

            move(2, Direction.RIGHT);

            //Deposit glyph
            glyphStopper.setPosition(GLYPH_STOPPER_UP);
            sleep(500);
            glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
            sleep(1000);

            move(3, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up
        }
    }
}
