package org.firstinspires.ftc.teamcode.RelicRecovery.Worlds;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/*
This code is the code for the blue center cryptobox.
The robot hits the jewel, reads the VuMark, and drives off of the balancing stone.
 It then drives to the correct cryptobox column and deposits the jewel.
 */
@Autonomous(name = "Blue Center Worlds", group = "Worlds")
public class BlueCenterAutonomous extends OpModeBase {
    public void runOpMode() {
        super.runOpMode(OpModeType.AUTONOMOUS);

        hitJewel("Blue");

        //Read VuMark to determine cryptobox key
        RelicRecoveryVuMark vuMark = readVuMark("Blue");

        move(13, Direction.FORWARD);
        turn(55, Direction.RIGHT);

        if(vuMark.equals(RelicRecoveryVuMark.LEFT)) {
            move(20, Direction.BACKWARD);
            turn(80, Direction.LEFT);
            move(4, Direction.BACKWARD);
        } else if(vuMark.equals(RelicRecoveryVuMark.CENTER)) {
            move(12, Direction.BACKWARD);
            turn(31, Direction.LEFT);
            move(7.5, Direction.BACKWARD);
        } else if(vuMark.equals(RelicRecoveryVuMark.RIGHT)) {
            move(12, Direction.BACKWARD);
            turn(20, Direction.LEFT);
            move(9, Direction.BACKWARD);
        }

        flipGlyph();
    }
}
