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
            move(11, Direction.LEFT); //Strafe to align with column
            turn(135, Direction.RIGHT, 1); //Reverse robot

            //Deposit glyph
            glyphStopper.setPosition(GLYPH_STOPPER_UP);
            sleep(500);
            glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
            sleep(1000);

            move(3, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up
        } else if(vuMark.equals(RelicRecoveryVuMark.CENTER)) {
            move(28, Direction.FORWARD);
            move(5.5, Direction.LEFT); //Strafe to align with column
            turn(135, Direction.RIGHT, 1); //Reverse robot

            //Deposit glyph
            glyphStopper.setPosition(GLYPH_STOPPER_UP);
            sleep(500);
            glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
            sleep(1000);

            move(3, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up
        } else if(vuMark.equals(RelicRecoveryVuMark.RIGHT)) {
            move(28, Direction.FORWARD);
            move(2.5, Direction.RIGHT); //Strafe to align with column
            turn(135, Direction.RIGHT, 1); //Reverse robot

            move(1.5, Direction.BACKWARD); //Move toward cryptobox before dumping glyph

            //Deposit glyph
            glyphStopper.setPosition(GLYPH_STOPPER_UP);
            sleep(500);
            glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
            sleep(1000);

            move(3, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up
        }
    }
}
