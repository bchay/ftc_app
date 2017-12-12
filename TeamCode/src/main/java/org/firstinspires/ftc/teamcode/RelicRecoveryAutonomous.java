package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

import java.nio.channels.ClosedByInterruptException;

@Autonomous(name = "Relic Recovery Autonomous")
public class RelicRecoveryAutonomous extends OpModeBase { //MecanumTeleop is a LinearOpMode so it can extend the same base class as autonomous
    VuMarkReader vuMarkReader;
    private ElapsedTime time; //Used for sensor reading timeout

    public void runOpMode() {
        super.runOpMode(RelicRecoveryAutonomous.class);

        vuMarkReader = new VuMarkReader(hardwareMap);
        time = new ElapsedTime();

        while(!isStarted() && !isStopRequested()) { //Display distance telemetry for robot alignment
            telemetry.addData("Ready to start the program.", "");
            telemetry.addData("Distance", range.getDistance(DistanceUnit.CM));
            telemetry.addData("Alliance color", allianceColor);
            telemetry.addData("Location", location);
            telemetry.addData("Delay", delay);
            telemetry.addData("Encoders", motorLeft1.getCurrentPosition() + motorLeft2.getCurrentPosition() + motorRight1.getCurrentPosition() + motorRight2.getCurrentPosition());
            telemetry.addData("Heading", getIntegratedHeading());
            telemetry.addData("VuMark", vuMarkReader.getVuMark());
            telemetry.update();
        }

        //No need for waitForStart because of while(!isStarted()) loop

        //Grab preloaded glyph
        leftGlyphGrabber.setPosition(LEFT_GLYPH_GRABBR_CLOSED);
        rightGlyphGrabber.setPosition(RIGHT_GLYPH_GRABBR_CLOSED);
        sleep(800); //Give servos time to move down
        glyphLift.setPower(-1); //Move lift up
        sleep(500);
        glyphLift.setPower(0);


        arm.setPosition(COLOR_SENSOR_ARM_OUT);
        sleep(800);

        time.reset();
        while(getColor().equals("Unknown") && opModeIsActive() && time.milliseconds() < 1000) { //Timeout at 1 second
            telemetry.addData("Color", "Unknown");
            telemetry.update();
        }

        //Color sensor reads left jewel

        if(getColor().equals(allianceColor)) {
            move(-4.5);
            arm.setPosition(COLOR_SENSOR_ARM_IN);
            move(4.5);
        } else if(!getColor().equals("Unknown")) { //Color is still detected
            move(4.5);
            arm.setPosition(COLOR_SENSOR_ARM_IN);
            move(-4.5);
        } else { //Color could not be detected; move arm in to avoid it hitting the wall
            arm.setPosition(COLOR_SENSOR_ARM_IN);
            sleep(1000);
        }

        RelicRecoveryVuMark vuMark = vuMarkReader.getVuMark();

        time.reset();
        while(vuMark.equals(RelicRecoveryVuMark.UNKNOWN) && opModeIsActive() && time.milliseconds() < 1000) { //Loop until VuMark is detected, timeout after 1 second
            vuMark = vuMarkReader.getVuMark();
            telemetry.addData("Determining VuMark", vuMark);
            telemetry.update();
        }

        //Default to close cryptobox column
        if(vuMark.equals(RelicRecoveryVuMark.UNKNOWN)) vuMark = allianceColor.equals("Blue") ? RelicRecoveryVuMark.LEFT : RelicRecoveryVuMark.RIGHT;

        if(location.equals("Side")) {
            move((allianceColor.equals("Blue") ? -1 : 1) * 26);
            turn(90, allianceColor.equals("Blue") ? Direction.RIGHT : Direction.LEFT);

            if(vuMark.equals(RelicRecoveryVuMark.RIGHT)) move(allianceColor.equals("Blue") ? -17 : 3, moveSpeedMax, false);
            else if(vuMark.equals(RelicRecoveryVuMark.CENTER)) move(allianceColor.equals("Blue") ? -11 : 11, moveSpeedMax, false);
            else if(vuMark.equals(RelicRecoveryVuMark.LEFT)) move(allianceColor.equals("Blue") ? -3 : 20, moveSpeedMax, false);

            turn(90, allianceColor.equals("Blue") ? Direction.RIGHT : Direction.LEFT);
        } else {
            if(vuMark.equals(RelicRecoveryVuMark.RIGHT)) move(allianceColor.equals("Blue") ? -46 : 30, moveSpeedMax, false);
            else if(vuMark.equals(RelicRecoveryVuMark.CENTER)) move(allianceColor.equals("Blue") ? -37.5 : 37.5, moveSpeedMax, false);
            else if(vuMark.equals(RelicRecoveryVuMark.LEFT)) move(allianceColor.equals("Blue") ? -30 : 46, moveSpeedMax, false);

            turn(90, Direction.RIGHT, turnSpeed - .1, 0, 5000);
        }

        move(8, moveSpeedMax, false, 0, 2000); //Move forward into cryptobox

        glyphLift.setPower(1); //Move lift down
        sleep(400);
        glyphLift.setPower(0);

        sleep(200);

        leftGlyphGrabber.setPosition(LEFT_GLYPH_GRABBR_OPEN);
        rightGlyphGrabber.setPosition(RIGHT_GLYPH_GRABBR_OPEN);
        sleep(1200);

        move(-5, moveSpeedMax);

        leftGlyphGrabber.setPosition(LEFT_GLYPH_GRABBR_CLOSED); //Move servos into closed position before driving into glyph
        rightGlyphGrabber.setPosition(RIGHT_GLYPH_GRABBR_CLOSED);
        sleep(1000);

        move(7, moveSpeedMax, false, 0, 2000); //Timeout because robot may hit crptobox
        move(-8, moveSpeedMax);
    }

}