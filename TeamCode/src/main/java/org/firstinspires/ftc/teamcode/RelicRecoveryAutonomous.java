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


        colorSensorArm.setPosition(.25); //Slightly down
        sleep(500);
        colorSensorRotator.setPosition(.532); //Center arm between jewels
        sleep(500);
        colorSensorArm.setPosition(.115); //Move down next to right jewel
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
            colorSensorRotator.setPosition(.139); //Move to hit left jewel
            sleep(500);
        } else if(!getColor().equals("Unknown")) { //Color is still detected, is opposing alliance's color
            colorSensorRotator.setPosition(.806); //Move to hit right jewel
            sleep(500);
        } else { //Color not detected, move arm up and right
            colorSensorArm.setPosition(.25);
            sleep(500);
            colorSensorRotator.setPosition(.806);
            sleep(500);
        }

        //Return jewel arm to upright position so that it does not get in the way of the remainder of the autonomous
        colorSensorArm.setPosition(1); //Move arm up
        sleep(3500);
        colorSensorRotator.setPosition(.372); //Move rotator behind metal piece to stop it from falling after teleop ends
        sleep(500);

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
            if(vuMark.equals(RelicRecoveryVuMark.LEFT)) {
                move(28, Direction.FORWARD);
                move(15.5, Direction.LEFT); //Strafe to left column

                turn(150, Direction.RIGHT, .95, 2, 10000); //Reverse robot

                move(2, Direction.BACKWARD);

                //Deposit glyph
                glyphStopper.setPosition(GLYPH_STOPPER_UP);
                sleep(500);
                glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
                sleep(1000);

                move(5, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up
            } else if(vuMark.equals(RelicRecoveryVuMark.CENTER)) {
                move(28, Direction.FORWARD);
                move(9, Direction.LEFT); //Strafe to align with column
                turn(153, Direction.RIGHT, .95); //Reverse robot

                //Deposit glyph
                glyphStopper.setPosition(GLYPH_STOPPER_UP);
                sleep(500);
                glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
                sleep(1000);

                move(5, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up
            } else if(vuMark.equals(RelicRecoveryVuMark.RIGHT)) {
                move(28, Direction.FORWARD);
                turn(150, Direction.RIGHT); //Reverse robot

                move(2, Direction.RIGHT);

                //Deposit glyph
                glyphStopper.setPosition(GLYPH_STOPPER_UP);
                sleep(500);
                glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
                sleep(1000);

                move(3, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up
            }
        } else if(location.equals("Side") && allianceColor.equals("Blue")) {
            move(31, Direction.BACKWARD);

            if(vuMark.equals(RelicRecoveryVuMark.RIGHT)) {
                move(17, Direction.LEFT); //Strafe toward right column

                turn(23, Direction.RIGHT);
                move(2, Direction.BACKWARD, moveSpeedMax, false, 1000); //Move toward cryptobox

                //Deposit glyph
                glyphStopper.setPosition(GLYPH_STOPPER_UP);
                sleep(500);
                glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
                sleep(1000);

                move(4, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up
            } else if(vuMark.equals(RelicRecoveryVuMark.CENTER)) {
                move(7.5, Direction.LEFT);

                turn(25, Direction.RIGHT);
                move(1.5, Direction.BACKWARD, moveSpeedMax, false, 1000); //Move toward cryptobox

                //Deposit glyph
                glyphStopper.setPosition(GLYPH_STOPPER_UP);
                sleep(500);
                glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
                sleep(1000);

                move(4, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up
            } else if(vuMark.equals(RelicRecoveryVuMark.LEFT)) {
                move(2.5, Direction.LEFT);

                turn(19, Direction.RIGHT);
                move(1, Direction.FORWARD); //Back up before dumping glyph

                //Deposit glyph
                glyphStopper.setPosition(GLYPH_STOPPER_UP);
                sleep(500);
                glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
                sleep(1000);

                move(5, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up
            }
        } else if(location.equals("Center") && allianceColor.equals("Red")) {
            if(vuMark.equals(RelicRecoveryVuMark.RIGHT)) {
                move(25, Direction.FORWARD);

                turn(105, Direction.LEFT); //Turn so that back of robot is facing cryptobox
                move(2, Direction.BACKWARD, moveSpeedMax, false, 1000); //Move away from cryptobox


                //Deposit glyph
                glyphStopper.setPosition(GLYPH_STOPPER_UP);
                sleep(500);
                glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
                sleep(1000);

                move(4, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up
            } else if(vuMark.equals(RelicRecoveryVuMark.CENTER)) {
                move(29, Direction.FORWARD);

                turn(120, Direction.LEFT, .8); //Turn so that back of robot is facing cryptobox
                move(3.5, Direction.BACKWARD, moveSpeedMax, false, 1000); //Move towards cryptobox

                //Deposit glyph
                glyphStopper.setPosition(GLYPH_STOPPER_UP);
                sleep(400);
                glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
                sleep(600);

                move(4.5, Direction.FORWARD, moveSpeedMax, false, 1000); //Move away from cryptobox
            } else if(vuMark.equals(RelicRecoveryVuMark.LEFT)) {
                sleep(1000);
                move(34.5, Direction.FORWARD);

                turn(125, Direction.LEFT); //Turn so that back of robot is facing cryptobox
                move(5, Direction.BACKWARD, moveSpeedMax, false, 1000); //Move closer to cryptobox

                //Deposit glyph
                glyphStopper.setPosition(GLYPH_STOPPER_UP);
                sleep(500);
                glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
                sleep(1000);

                move(4, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up
            }
        } else if(location.equals("Center") && allianceColor.equals("Blue")) {
            if(vuMark.equals(RelicRecoveryVuMark.RIGHT)) {
                move(34.5, Direction.BACKWARD);
                turn(56, Direction.LEFT); //Turn so that back of robot is facing cryptobox

                move(5.5, Direction.BACKWARD); //Move towards cryptobox

                //Deposit glyph
                glyphStopper.setPosition(GLYPH_STOPPER_UP);
                sleep(500);
                glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
                sleep(1000);

                move(3, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up
            } else if(vuMark.equals(RelicRecoveryVuMark.CENTER)) {
                move(22, Direction.BACKWARD);

                turn(48, Direction.LEFT); //Turn so that back of robot is facing cryptobox
                move(6.5, Direction.BACKWARD, moveSpeedMax, false, 1000); //Move toward cryptobox before dumping glyph

                //Deposit glyph
                glyphStopper.setPosition(GLYPH_STOPPER_UP);
                sleep(500);
                glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
                sleep(1000);

                move(4, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up
            } else if(vuMark.equals(RelicRecoveryVuMark.LEFT)) {
                move(35, Direction.BACKWARD);
                turn(118, Direction.LEFT); //Turn so that back of robot is facing cryptobox

                move(4, Direction.BACKWARD); //Move towards cryptobox

                //Deposit glyph
                glyphStopper.setPosition(GLYPH_STOPPER_UP);
                sleep(500);
                glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
                sleep(1000);

                move(3, Direction.FORWARD, moveSpeedMax, false, 1000); //Back up
            }
        }
    }
}