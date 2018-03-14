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

        //Move intake down by spinning wheels
        leftIntake.setPower(1);
        rightIntake.setPower(-1);
        sleep(800);

        leftIntake.setPower(0);
        rightIntake.setPower(0);

        //Slowly move flipper up tp deposit glyph into cryptobox
        while(opModeIsActive() && Math.abs(leftFlipper.getPosition() - LEFT_FLIPPER_UP) > .01) {
            leftFlipper.setPosition(Range.clip(leftFlipper.getPosition() + .008, 0, LEFT_FLIPPER_UP));
            rightFlipper.setPosition(Range.clip(rightFlipper.getPosition() - .008, RIGHT_FLIPPER_UP, 1));
        }

        leftFlipper.setPosition(LEFT_FLIPPER_UP);
        rightFlipper.setPosition(RIGHT_FLIPPER_UP);

        move(5, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up
    }
}
