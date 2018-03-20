package org.firstinspires.ftc.teamcode.RelicRecovery.NSR;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name = "Blue Center NSR", group = "NSR")
public class BlueCenterAutonomous extends OpModeBase {
    public void runOpMode() {
        super.runOpMode(OpModeType.AUTONOMOUS);
        hitJewel("Blue");

        //Read VuMark to determine cryptobox key
        RelicRecoveryVuMark vuMark = readVuMark("Blue");


        //Autonomous movement code
        if(vuMark.equals(RelicRecoveryVuMark.RIGHT)) {
            move(30, Direction.BACKWARD);
            turn(50, Direction.LEFT); //Turn so that back of robot is facing cryptobox
            move(9, Direction.BACKWARD); //Move towards cryptobox
        } else if(vuMark.equals(RelicRecoveryVuMark.CENTER)) {
            move(45.5, Direction.BACKWARD);
            turn(120, Direction.LEFT); //Turn so that back of robot is facing cryptobox
            move(5.5, Direction.BACKWARD); //Move towards cryptobox
        } else if(vuMark.equals(RelicRecoveryVuMark.LEFT)) {
            move(38, Direction.BACKWARD);
            turn(120, Direction.LEFT); //Turn so that back of robot is facing cryptobox
            move(7.5, Direction.BACKWARD); //Move towards cryptobox
        }

        flipGlyph();
    }
}
