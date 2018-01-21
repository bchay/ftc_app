package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

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
            telemetry.addData("Alliance color", allianceColor);
            telemetry.addData("Location", location);
            telemetry.addData("Encoders", motorLeftFront.getCurrentPosition() + motorLeftBack.getCurrentPosition() + motorRightFront.getCurrentPosition() + motorRightBack.getCurrentPosition());
            telemetry.addData("Heading", getIntegratedHeading());
            telemetry.addData("VuMark", vuMarkReader.getVuMark());
            telemetry.update();
        }


        colorSensorArm.setPosition(.197); //Slightly down
        sleep(500);
        colorSensorRotator.setPosition(.511); //Centered forward
        sleep(500);
        colorSensorArm.setPosition(.118); //Move down next to right jewel
        sleep(500);


        //Knock off jewel of opposing alliance color
        time.reset();
        while(getColor().equals("Unknown") && opModeIsActive() && time.milliseconds() < 1000) { //Timeout at 1 second
            telemetry.addData("Color", "Unknown");
            telemetry.update();
        }

        //Color sensor reads left jewel
        telemetry.addData("Color", getColor());
        telemetry.update();

        if(!getColor().equals(allianceColor) && !getColor().equals("Unknown")) {
            colorSensorRotator.setPosition(.806); //Move to hit left jewel
            sleep(500);
        } else if(!getColor().equals("Unknown")) { //Color is still detected, is opposing alliance's color
            colorSensorRotator.setPosition(.139); //Move to hit right jewel
            sleep(500);
        }

        colorSensorArm.setPosition(.197); //Move arm up slightly
        sleep(500);
        colorSensorRotator.setPosition(COLOR_ROTATOR_INITIAL); //Move arm right
        sleep(500);
        colorSensorArm.setPosition(COLOR_SENSOR_ARM_INITIAL); //Move arm up
        sleep(500); //No delay, time will be spent reading the VuMark


        /*
        //Read VuMark to determine cryptobox key
        RelicRecoveryVuMark vuMark = vuMarkReader.getVuMark();

        time.reset();
        while(vuMark.equals(RelicRecoveryVuMark.UNKNOWN) && opModeIsActive() && time.milliseconds() < 1000) { //Loop until VuMark is detected, timeout after 1 second
            vuMark = vuMarkReader.getVuMark();
            telemetry.addData("Determining VuMark", vuMark);
            telemetry.update();
        }

        //Default to close cryptobox column if VuMark is not detected
        if(vuMark.equals(RelicRecoveryVuMark.UNKNOWN)) vuMark = allianceColor.equals("Blue") ? RelicRecoveryVuMark.LEFT : RelicRecoveryVuMark.RIGHT;

        if(location.equals("Side") && allianceColor.equals("Red")) {
            move(28, Direction.FORWARD);
            turn(180, Direction.LEFT, .95); //Reverse robot

            if(vuMark.equals(RelicRecoveryVuMark.LEFT)) {
                move(19.5, Direction.RIGHT); //Strafe to left column
                move(1, Direction.FORWARD, moveSpeedMax, false, 1000); //Move away from cryptobox so that glyph can fall in


                turn(25, Direction.LEFT);
                move(2, Direction.FORWARD, moveSpeedMax, false, 1000); //Move backward

                //Deposit glyph
                glyphStopper.setPosition(GLYPH_STOPPER_UP);
                sleep(500);
                glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
                sleep(1000);
                move(5, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up
                move(9, Direction.BACKWARD, moveSpeedMax, false, 1000); //Hit glyph again to push it into the cryptobox
                turn(10, vuMark.equals(RelicRecoveryVuMark.LEFT) ? Direction.RIGHT : Direction.LEFT, turnSpeed, 0, 1000);
                move(5, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up to avoid touching the glyph
            } else if(vuMark.equals(RelicRecoveryVuMark.CENTER)) {
                move(11.5, Direction.RIGHT);
                move(1, Direction.FORWARD, moveSpeedMax, false, 1000); //Move away from cryptobox so that glyph can fall in

                turn(25, Direction.LEFT);
                move(2, Direction.FORWARD, moveSpeedMax, false, 1000); //Move backward

                //Deposit glyph
                glyphStopper.setPosition(GLYPH_STOPPER_UP);
                sleep(500);
                glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
                sleep(1000);
                move(5, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up
                move(9, Direction.BACKWARD, moveSpeedMax, false, 1000); //Hit glyph again to push it into the cryptobox
                turn(10, Direction.RIGHT, turnSpeed, 0, 1000);
                move(5, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up to avoid touching the glyph

            } else if(vuMark.equals(RelicRecoveryVuMark.RIGHT)) {
                move(2, Direction.RIGHT, moveSpeedMax, false, 1000); //Strafe towards cryptobox
                move(1.5, Direction.BACKWARD, moveSpeedMax, false, 1000); //Strafe towards center of cryptobox

                turn(23, Direction.LEFT);
                move(2, Direction.FORWARD, moveSpeedMax, false, 1000); //Move backward

                //Deposit glyph
                glyphStopper.setPosition(GLYPH_STOPPER_UP);
                sleep(500);
                glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
                sleep(1000);
                move(5, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up
            }
        } else if(location.equals("Side") && allianceColor.equals("Blue")) {
            move(31, Direction.BACKWARD);

            if(vuMark.equals(RelicRecoveryVuMark.RIGHT)) {
                move(14, Direction.LEFT);


                turn(25, Direction.RIGHT);
                move(1, Direction.FORWARD, moveSpeedMax, false, 1000); //Back away from cryptobox

                //Deposit glyph
                glyphStopper.setPosition(GLYPH_STOPPER_UP);
                sleep(500);
                glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
                sleep(1000);
                move(5, Direction.FORWARD, moveSpeedMax, false, 1000);
                move(8, Direction.BACKWARD, moveSpeedMax, false, 1000);
                turn(15, vuMark.equals(RelicRecoveryVuMark.LEFT) ? Direction.RIGHT : Direction.LEFT, turnSpeed, 0, 1000);
                move(5, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up to avoid touching the glyph
            } else if(vuMark.equals(RelicRecoveryVuMark.CENTER)) {
                move(6, Direction.LEFT);


                turn(25, Direction.RIGHT);
                move(1, Direction.FORWARD, moveSpeedMax, false, 1000); //Back away from cryptobox

                //Deposit glyph
                glyphStopper.setPosition(GLYPH_STOPPER_UP);
                sleep(500);
                glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
                sleep(1000);
                move(5, Direction.FORWARD, moveSpeedMax, false, 1000);
                move(8, Direction.BACKWARD, moveSpeedMax, false, 1000);
                turn(15, vuMark.equals(RelicRecoveryVuMark.LEFT) ? Direction.RIGHT : Direction.LEFT, turnSpeed, 0, 1000);
                move(5, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up to avoid touching the glyph
            } else if(vuMark.equals(RelicRecoveryVuMark.LEFT)) {
                turn(25, Direction.RIGHT);
                move(2.5, Direction.FORWARD, moveSpeedMax, false, 1000); //Back away from cryptobox

                //Deposit glyph
                glyphStopper.setPosition(GLYPH_STOPPER_UP);
                sleep(500);
                glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
                sleep(1000);
                move(5, Direction.FORWARD, moveSpeedMax, false, 1000);
                move(8, Direction.BACKWARD, moveSpeedMax, false, 1000);
                turn(15, Direction.LEFT, turnSpeed, 0, 1000);
                move(5, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up to avoid touching the glyph
            }
        } else if(location.equals("Center") && allianceColor.equals("Red")) {
            if(vuMark.equals(RelicRecoveryVuMark.RIGHT)) {
                move(24, Direction.FORWARD);

                turn(105, Direction.LEFT); //Turn so that back of robot is facing cryptobox
                move(2, Direction.BACKWARD, moveSpeedMax, false, 1000);


                //Deposit glyph
                glyphStopper.setPosition(GLYPH_STOPPER_UP);
                sleep(500);
                glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
                sleep(1000);
                move(5, Direction.FORWARD, moveSpeedMax, false, 1000);
                move(8, Direction.BACKWARD, moveSpeedMax, false, 1000);
                turn(15, Direction.RIGHT, turnSpeed, 0, 1000);
                move(5, Direction.FORWARD, moveSpeedMax, false, 1000);
                //move(8, Direction.BACKWARD, moveSpeedMax, false, 1000);
                //move(8, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up to avoid touching the glyph
            } else if(vuMark.equals(RelicRecoveryVuMark.CENTER)) {
                move(26, Direction.FORWARD);

                turn(120, Direction.LEFT); //Turn so that back of robot is facing cryptobox
                move(2, Direction.BACKWARD, moveSpeedMax, false, 1000);


                //Deposit glyph
                glyphStopper.setPosition(GLYPH_STOPPER_UP);
                sleep(500);
                glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
                sleep(1000);
                move(5, Direction.FORWARD, moveSpeedMax, false, 1000);
                move(8, Direction.BACKWARD, moveSpeedMax, false, 1000);
                turn(15, Direction.RIGHT, turnSpeed, 0, 1000);
                move(5, Direction.FORWARD, moveSpeedMax, false, 1000);
                //move(8, Direction.BACKWARD, moveSpeedMax, false, 1000);
                //move(8, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up to avoid touching the glyph
            } else if(vuMark.equals(RelicRecoveryVuMark.LEFT)) {
                move(32.5, Direction.FORWARD);

                turn(125, Direction.LEFT); //Turn so that back of robot is facing cryptobox
                move(4, Direction.BACKWARD, moveSpeedMax, false, 1000); //Move closer to cryptobox

                //Deposit glyph
                glyphStopper.setPosition(GLYPH_STOPPER_UP);
                sleep(500);
                glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
                sleep(1000);
                move(5, Direction.FORWARD, moveSpeedMax, false, 1000);
                move(8, Direction.BACKWARD, moveSpeedMax, false, 1000);
                turn(15, Direction.RIGHT, turnSpeed, 0, 1000);
                move(5, Direction.FORWARD, moveSpeedMax, false, 1000);
                //move(8, Direction.BACKWARD, moveSpeedMax, false, 1000);
                //move(8, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up to avoid touching the glyph
            }

        } else if(location.equals("Center") && allianceColor.equals("Blue")) {
            if(vuMark.equals(RelicRecoveryVuMark.RIGHT)) {
                move(35, Direction.BACKWARD);

                turn(45, Direction.LEFT); //Turn so that back of robot is facing cryptobox

                //Deposit glyph
                glyphStopper.setPosition(GLYPH_STOPPER_UP);
                sleep(500);
                glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
                sleep(1000);
                move(5, Direction.FORWARD, moveSpeedMax, false, 1000);
                move(8, Direction.BACKWARD, moveSpeedMax, false, 1000);
                turn(15, Direction.LEFT, turnSpeed, 0, 1000);
                move(5, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up to avoid touching the glyph
//              move(8, Direction.BACKWARD, moveSpeedMax, false, 1000);
//              move(8, Direction.FORWARD, moveSpeedMax, false, 1000);

            } else if(vuMark.equals(RelicRecoveryVuMark.CENTER)) {
                move(23, Direction.BACKWARD);

                turn(45, Direction.LEFT); //Turn so that back of robot is facing cryptobox
                move(4, Direction.BACKWARD, moveSpeedMax, false, 1000);

                //Deposit glyph
                glyphStopper.setPosition(GLYPH_STOPPER_UP);
                sleep(500);
                glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
                sleep(1000);
                move(5, Direction.FORWARD, moveSpeedMax, false, 1000);
                move(8, Direction.BACKWARD, moveSpeedMax, false, 1000);
                turn(15, Direction.LEFT, turnSpeed, 0, 1000);
                move(5, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up to avoid touching the glyph
                //move(8, Direction.BACKWARD, moveSpeedMax, false, 1000);
                //move(8, Direction.FORWARD, moveSpeedMax, false, 1000);

            } else if(vuMark.equals(RelicRecoveryVuMark.LEFT)) {
                move(35, Direction.BACKWARD);
                turn(115, Direction.LEFT); //Turn so that back of robot is facing cryptobox

                //Deposit glyph
                glyphStopper.setPosition(GLYPH_STOPPER_UP);
                sleep(500);
                glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
                sleep(1000);
                move(2, Direction.FORWARD, moveSpeedMax, false, 1000);
                move(8, Direction.BACKWARD, moveSpeedMax, false, 1000);
                turn(15, Direction.RIGHT, turnSpeed, 0, 1000);
                move(5, Direction.FORWARD, moveSpeedMax, false, 1000);
                //move(8, Direction.BACKWARD, moveSpeedMax, false, 1000);
                //move(8, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up to avoid touching the glyph
            }
        }

        glyphFlipper.setPosition(GLYPH_FLIPPER_FLAT);
        sleep(1000);
        glyphStopper.setPosition(GLYPH_STOPPER_DOWN);
        sleep(500);
        */

    }
}