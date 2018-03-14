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
        leftFlipper.setPosition(LEFT_FLIPPER_UP); //Ensure that flipper is fully up because of Math.abs threshold
        rightFlipper.setPosition(RIGHT_FLIPPER_UP);
        led.setPower(-1);

        move(4, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up

        leftFlipper.setPosition(LEFT_FLIPPER_DOWN); //Move flipper into robot before ramming back into glyph
        rightFlipper.setPosition(RIGHT_FLIPPER_DOWN);
        sleep(500);

        move(7, Direction.BACKWARD, moveSpeedMax, false, 1000); //Hit glyph again, pushing it into cryptobox
        move(5, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up
    }
}
