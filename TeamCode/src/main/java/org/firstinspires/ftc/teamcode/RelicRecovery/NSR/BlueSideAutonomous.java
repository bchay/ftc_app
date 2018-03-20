package org.firstinspires.ftc.teamcode.RelicRecovery.NSR;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name = "Blue Side NSR", group = "NSR")
public class BlueSideAutonomous extends OpModeBase {
    public void runOpMode() {
        super.runOpMode(OpModeType.AUTONOMOUS);
        hitJewel("Blue");

        //Read VuMark to determine cryptobox key
        RelicRecoveryVuMark vuMark = readVuMark("Blue");

        //Autonomous movement code
        if(vuMark.equals(RelicRecoveryVuMark.RIGHT)) {
            move(30, Direction.BACKWARD); //Drive off balancing stone toward cryptobox
            move(27.5, Direction.LEFT); //Strafe toward right column
            turn(43, Direction.LEFT); //Turn toward right column
            move(4, Direction.BACKWARD); //Move towards cryptobox before dumping glyph
        } else if(vuMark.equals(RelicRecoveryVuMark.CENTER)) {
            move(27.7, Direction.BACKWARD); //Drive off balancing stone toward cryptobox
            move(21, Direction.LEFT); //Strafe toward right side of the cryptobox
            turn(35, Direction.LEFT);
            move(5, Direction.BACKWARD, moveSpeedMax, false, 1000); //Move toward cryptobox
        } else if(vuMark.equals(RelicRecoveryVuMark.LEFT)) {
            move(27.6, Direction.BACKWARD); //Drive off balancing stone toward cryptobox
            move(14, Direction.LEFT); //Strafe to the right side of the cryptobox
            turn(40, Direction.LEFT);
            move(5, Direction.BACKWARD, moveSpeedMax, false, 1000); //Move toward cryptobox
        }


       flipGlyph();
    }
}
