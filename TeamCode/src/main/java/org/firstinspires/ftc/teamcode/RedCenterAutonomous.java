package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name = "Red Center", group = "Autonomous")
public class RedCenterAutonomous extends OpModeBase {
    public void runOpMode() {
        super.runOpMode(OpModeType.AUTONOMOUS);
        hitJewel("Red");

        //Read VuMark to determine cryptobox key
        RelicRecoveryVuMark vuMark = readVuMark("Red");

        if(vuMark.equals(RelicRecoveryVuMark.RIGHT)) {
            move(39.66, Direction.FORWARD);
            turn(55, Direction.LEFT); //Turn so that back of robot is facing cryptobox
            move(7.5, Direction.BACKWARD); //Move towards cryptobox

            //Deposit glyph
            glyphStopper.setPosition(GLYPH_STOPPER_UP);
            sleep(500);
            glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
            sleep(1000);

            move(4, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up
        } else if(vuMark.equals(RelicRecoveryVuMark.CENTER)) {
            move(24, Direction.FORWARD);

            turn(130, Direction.LEFT, .8); //Turn so that back of robot is facing cryptobox
            move(5, Direction.BACKWARD, moveSpeedMax, false, 1000); //Move towards cryptobox before dumping glyph

            //Deposit glyph
            glyphStopper.setPosition(GLYPH_STOPPER_UP);
            sleep(400);
            glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
            sleep(600);

            move(4.5, Direction.FORWARD, moveSpeedMax, false, 1000); //Move away from cryptobox
        } else if(vuMark.equals(RelicRecoveryVuMark.LEFT)) {
            move(30.5, Direction.FORWARD);

            turn(130, Direction.LEFT, .8); //Turn so that back of robot is facing cryptobox
            move(6, Direction.BACKWARD, moveSpeedMax, false, 1000); //Move towards cryptobox before dumping glyph

            //Deposit glyph
            glyphStopper.setPosition(GLYPH_STOPPER_UP);
            sleep(400);
            glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
            sleep(600);

            move(4.5, Direction.FORWARD, moveSpeedMax, false, 1000); //Move away from cryptobox
        }
    }
}
