package org.firstinspires.ftc.teamcode.RelicRecovery.Worlds;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/*
This code is the code for the red side cryptobox.
The robot hits the jewel, reads the VuMark, and drives off of the balancing stone.
 It then drives to the correct cryptobox column and deposits the jewel.
 */
@Autonomous(name = "Red Side Worlds", group = "Worlds")
public class RedSideAutonomous extends OpModeBase {
    public void runOpMode() {
        super.runOpMode(OpModeType.AUTONOMOUS);

        hitJewel("Red");

        //Read VuMark to determine cryptobox key
        RelicRecoveryVuMark vuMark = readVuMark("Red");

        move(14.5, Direction.FORWARD); //Drive off of balancing stone

        if(vuMark.equals(RelicRecoveryVuMark.LEFT)) {
            turn(70, Direction.LEFT);
            move(13.5, Direction.BACKWARD);
        } else if(vuMark.equals(RelicRecoveryVuMark.CENTER)) {
            turn(63, Direction.LEFT);
            move(15, Direction.BACKWARD);
        } else if(vuMark.equals(RelicRecoveryVuMark.RIGHT)) {
            turn(53, Direction.LEFT);
            move(16.5, Direction.BACKWARD);
        }

        flipGlyph();
    }
}
