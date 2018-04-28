package org.firstinspires.ftc.teamcode.RelicRecovery.NSR;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name = "Red Side NSR", group = "NSR")
@Disabled
public class RedSideAutonomous extends OpModeBase {
    public void runOpMode() {
        super.runOpMode(OpModeType.AUTONOMOUS);
        hitJewel("Red");

        //Read VuMark to determine cryptobox key
        RelicRecoveryVuMark vuMark = readVuMark("Red");

        //Autonomous movement code
        if(vuMark.equals(RelicRecoveryVuMark.LEFT)) {
            move(28, Direction.FORWARD);
            move(27.5, Direction.LEFT); //Strafe to align with column
            turn(150, Direction.LEFT, 1); //Reverse robot
            move(3.5, Direction.BACKWARD); //Move closer to cryptobox
        } else if(vuMark.equals(RelicRecoveryVuMark.CENTER)) {
            move(28, Direction.FORWARD);
            move(18.5, Direction.LEFT); //Strafe to align with column
            turn(150, Direction.LEFT, 1); //Reverse robot
            move(3, Direction.BACKWARD);
        } else if(vuMark.equals(RelicRecoveryVuMark.RIGHT)) {
            move(28, Direction.FORWARD);
            move(13, Direction.LEFT); //Strafe to align with column
            turn(150, Direction.LEFT, 1); //Reverse robot
            move(3.5, Direction.BACKWARD);
        }

        flipGlyph();
    }
}
