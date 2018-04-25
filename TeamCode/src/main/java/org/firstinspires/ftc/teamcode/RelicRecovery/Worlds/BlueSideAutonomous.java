package org.firstinspires.ftc.teamcode.RelicRecovery.Worlds;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/*
This code is the code for the blue side cryptobox.
The robot hits the jewel, reads the VuMark, and drives off of the balancing stone.
 It then drives to the correct cryptobox column and deposits the jewel.
 */
@Autonomous(name = "Blue Side Worlds", group = "Worlds")
public class BlueSideAutonomous extends OpModeBase {
    public void runOpMode() {
        super.runOpMode(OpModeType.AUTONOMOUS);

        hitJewel("Blue");

        //Read VuMark to determine cryptobox key
        RelicRecoveryVuMark vuMark = readVuMark("Blue");

        move(14.5, Direction.FORWARD); //Drive off of balancing stone

        if(vuMark.equals(RelicRecoveryVuMark.RIGHT)) {
            turn(61, Direction.RIGHT);
            move(14, Direction.BACKWARD);
        } else if(vuMark.equals(RelicRecoveryVuMark.CENTER)) {
            turn(66, Direction.RIGHT);
            move(15, Direction.BACKWARD);
        } else if(vuMark.equals(RelicRecoveryVuMark.LEFT)) {
            turn(53, Direction.RIGHT);
            move(17.5, Direction.BACKWARD);
        }

        flipGlyph();
    }
}
