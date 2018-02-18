package org.firstinspires.ftc.teamcode.RelicRecovery;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

//@Autonomous(name = "Blue Center", group = "Autonomous")
public class BlueCenterAutonomous extends OpModeBase {
    public void runOpMode() {
        super.runOpMode(OpModeType.AUTONOMOUS);
        hitJewel("Blue");

        //Read VuMark to determine cryptobox key
        RelicRecoveryVuMark vuMark = readVuMark("Blue");


        //Autonomous movement code
        if(vuMark.equals(RelicRecoveryVuMark.RIGHT)) {
            move(34.5, Direction.BACKWARD);
            turn(56, Direction.LEFT); //Turn so that back of robot is facing cryptobox

            move(5.5, Direction.BACKWARD); //Move towards cryptobox

            //Deposit glyph
            glyphStopper.setPosition(GLYPH_STOPPER_UP);
            sleep(500);
            glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
            sleep(1000);

            move(3, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up
        } else if(vuMark.equals(RelicRecoveryVuMark.CENTER)) {
            move(22, Direction.BACKWARD);

            turn(48, Direction.LEFT); //Turn so that back of robot is facing cryptobox
            move(6.5, Direction.BACKWARD, moveSpeedMax, false, 1000); //Move toward cryptobox before dumping glyph

            //Deposit glyph
            glyphStopper.setPosition(GLYPH_STOPPER_UP);
            sleep(500);
            glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
            sleep(1000);

            move(4, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up
        } else if(vuMark.equals(RelicRecoveryVuMark.LEFT)) {
            move(35, Direction.BACKWARD);
            turn(118, Direction.LEFT); //Turn so that back of robot is facing cryptobox

            move(4, Direction.BACKWARD); //Move towards cryptobox

            //Deposit glyph
            glyphStopper.setPosition(GLYPH_STOPPER_UP);
            sleep(500);
            glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
            sleep(1000);

            move(3, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up
        }
    }
}
