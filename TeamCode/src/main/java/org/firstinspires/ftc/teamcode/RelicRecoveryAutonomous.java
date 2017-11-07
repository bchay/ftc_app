package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

@Autonomous(name = "Relic Recovery Autonomous")
public class RelicRecoveryAutonomous extends OpModeBase { //MecanumTeleop is a LinearOpMode so it can extend the same base class as autonomous
    VuMarkReader vuMarkReader;
    ElapsedTime time;

    public void runOpMode() {
        super.runOpMode();

        vuMarkReader = new VuMarkReader(hardwareMap);
        time = new ElapsedTime();

        waitForStart();

        lift.setPower(-1);
        sleep(400);
        lift.setPower(0);

        moveDirection = allianceColor.equals("Blue") ? Direction.RIGHT : Direction.LEFT;

        arm.setPosition(ARM_OUT);
        sleep(1000);

        time.reset();
        while(getColor() == 0 && opModeIsActive() && time.milliseconds() < 3000) { //Timeout at 1 second
            telemetry.addData("Color", "Unknown");
            telemetry.update();
        }

        //Color sensor reads right jewel
        if(getColor() == Color.RED && allianceColor.equals("Red")) {
            move(4.5);
            arm.setPosition(ARM_IN);
            move(-4);
        } else if(getColor() == Color.RED && allianceColor.equals("Blue")) {
            move(-4.5);
            arm.setPosition(ARM_IN);
            move(4);
        } else if(getColor() == Color.BLUE && allianceColor.equals("Blue")) {
            move(4.5);
            arm.setPosition(ARM_IN);
            move(-4);
        } else if(getColor() == Color.BLUE && allianceColor.equals("Red")) {
            move(-4.5);
            arm.setPosition(ARM_IN);
            move(4);
        } else {
            arm.setPosition(ARM_IN);
            sleep(1000);
        }

        RelicRecoveryVuMark vuMark = vuMarkReader.getVuMark();

        time.reset();
        while(vuMark.equals(RelicRecoveryVuMark.UNKNOWN) && opModeIsActive() && time.milliseconds() < 3000) { //Loop until VuMark is detected
            vuMark = vuMarkReader.getVuMark();
            telemetry.addData("Determining VuMark", vuMark);
        }

        telemetry.addData("VuMark", vuMark);
        telemetry.update();
        sleep(300);

        if(vuMark.equals(RelicRecoveryVuMark.UNKNOWN)) vuMark = RelicRecoveryVuMark.CENTER;


        if(location.equals("Side")) {
            move((allianceColor.equals("Blue") ? -1 : 1) * 26);
            turn(90, moveDirection);

            if(vuMark.equals(RelicRecoveryVuMark.RIGHT)) move(allianceColor.equals("Blue") ? -17 : 25, moveSpeedMax, false);
            else if(vuMark.equals(RelicRecoveryVuMark.CENTER)) move(allianceColor.equals("Blue") ? -20 : 20, moveSpeedMax, false);
            else if(vuMark.equals(RelicRecoveryVuMark.LEFT)) move(allianceColor.equals("Blue") ? -25 : 17, moveSpeedMax, false);

            turn(90, allianceColor.equals("Blue") ? moveDirection : moveDirection.next());
            move(8, moveSpeedMax, false);

            leftGlyphGrabber.setPosition(LEFT_GLYPH_GRABBR_OPEN);
            rightGlyphGrabber.setPosition(RIGHT_GLYPH_GRABBR_OPEN);
            sleep(1000);
            move(-8, moveSpeedMax, false);
        } else {
            if(vuMark.equals(RelicRecoveryVuMark.RIGHT)) move(allianceColor.equals("Blue") ? -44 : 29, moveSpeedMax, false);
            else if(vuMark.equals(RelicRecoveryVuMark.CENTER)) move(allianceColor.equals("Blue") ? -37 : 37, moveSpeedMax, false);
            else if(vuMark.equals(RelicRecoveryVuMark.LEFT)) move(allianceColor.equals("Blue") ? -29 : 44, moveSpeedMax, false);

            turn(90, allianceColor.equals("Blue") ? Direction.RIGHT : moveDirection.next());
            move(8, moveSpeedMax);
            leftGlyphGrabber.setPosition(LEFT_GLYPH_GRABBR_OPEN);
            rightGlyphGrabber.setPosition(RIGHT_GLYPH_GRABBR_OPEN);
            sleep(1000);

            move(-5, moveSpeedMax);
            move(5.5, moveSpeedMax);
            move(-8, moveSpeedMax);
        }

        lift.setPower(1);
        sleep(400);
        lift.setPower(0);
    }

}