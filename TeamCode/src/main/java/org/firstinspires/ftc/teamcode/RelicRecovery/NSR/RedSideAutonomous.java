package org.firstinspires.ftc.teamcode.RelicRecovery.NSR;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name = "Red Side NSR", group = "NSR")
public class RedSideAutonomous extends OpModeBase {
    public void runOpMode() {
        super.runOpMode(OpModeType.AUTONOMOUS);
        //hitJewel("Red");

        //Read VuMark to determine cryptobox key
        //RelicRecoveryVuMark vuMark = readVuMark("Red");
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.RIGHT;

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
        led.setPower(1);

        move(4, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up

        leftFlipper.setPosition(LEFT_FLIPPER_DOWN); //Move flipper into robot before ramming back into glyph
        rightFlipper.setPosition(RIGHT_FLIPPER_DOWN);
        sleep(500);

        move(7, Direction.BACKWARD, moveSpeedMax, false, 1000); //Hit glyph again, pushing it into cryptobox
        move(5, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up
    }
}
