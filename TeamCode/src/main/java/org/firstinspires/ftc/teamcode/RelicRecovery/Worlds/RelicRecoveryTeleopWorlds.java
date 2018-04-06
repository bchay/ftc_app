package org.firstinspires.ftc.teamcode.RelicRecovery.Worlds;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.TaskData;
import org.firstinspires.ftc.teamcode.ThreadTaskInterface;

import java.util.ArrayList;
import java.util.Date;
import java.util.HashMap;
import java.util.Iterator;

/*
Gamepad Mappings:

Gamepad 1:
    Left Joystick: Control robot movement in XY plane - robot moves in direction of joystick regardless of robot orientation
    Right Joystick: Robot turns to heading specified by joystick angle

    Right Trigger: Slowly moves flipper to PARTIALLY_UP position, and then sets position to FLIPPER_UP
    Right Bumper: Hold to release grabbers

    Left Bumper: Suck intake in
    Left Trigger: Suck intake out

    B: Turn on blue LED
    X: Turn on red LED

Gamepad 2:
    Right Joystick: Right intake - Backward joystick brings glyph in
    Left Joystick: Left intake - Backward joystick brings glyph in

    Right Bumper: Move glyph lever up - if not pressed, glyph lever will move down
    Left Bumper: Move flipper to FLIPPER_FLAT position

    Right Trigger: Move Glyph Lift up
    Left Trigger: Move Glyph Lift down

    D Pad up: Move color sensor arm
    D Pad Left: Move color sensor arm rotator
 */

/**
 * This is the code for the teleop program for the robot built for the FIRST World Championship competition.
 *
 */
@TeleOp(name = "Relic Recovery Teleop Worlds", group = "NSR")
public class RelicRecoveryTeleopWorlds extends OpModeBase {
    private ArrayList<TaskData> pendingTasks = new ArrayList<>();

    public void runOpMode() {
        super.runOpMode(OpModeType.TELEOP);
        boolean flag = true; //Used to determine the position of the glyph lever

        telemetry.addData("Ready to start program", "");
        telemetry.update();

        waitForStart();
        initializeServos(OpModeType.TELEOP); //Servos cannot be initialized during teleop init, only after start

        while (opModeIsActive()) {
            //********** Gamepad 1 - Start + A **********

            motorLeftFront.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x);
            motorLeftBack.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x);
            motorRightFront.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x);
            motorRightBack.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x);

            //Move glyph lever up at normal speed part of the way, and then slowly move it to the up position
            //Slowly move flipper up
            if(gamepad1.right_trigger > .5) {
                if(flag) { //Will only run the first time that this conditional executes
                    flag = false;

                    grabber.setPosition(GRABBER_CLOSED);

                    pendingTasks.add(new TaskData(600, new ThreadTaskInterface() {
                        @Override
                        public void runTask() {
                            glyphLever.setPosition(GLYPH_LEVER_BACK);
                        }
                    }));
                }

                //Move flipper up slowly for first part of upward movement
                if(leftFlipper.getPosition() > (LEFT_FLIPPER_DOWN - .01) && leftFlipper.getPosition() < LEFT_FLIPPER_PARTIALLY_UP) { //- .01 used to correct for floating point errors
                    leftFlipper.setPosition(Range.clip(leftFlipper.getPosition() + .005, 0, 1));
                    rightFlipper.setPosition(Range.clip(rightFlipper.getPosition() - .005, 0, 1));
                } else { //Move flipper fully up at faster speed
                    leftFlipper.setPosition(LEFT_FLIPPER_UP);
                    rightFlipper.setPosition(RIGHT_FLIPPER_UP);
                }
            } else if (!gamepad2.left_bumper) { //Do not override gamepad2 flipper movement
                //Slowly move flipper back down

                //Set grabber position to closed while moving flipper back down
                if(Math.abs(leftFlipper.getPosition() - LEFT_FLIPPER_UP) < .1) grabber.setPosition(GRABBER_CLOSED);

                leftFlipper.setPosition(Range.clip(leftFlipper.getPosition() - .005, LEFT_FLIPPER_DOWN, 1));
                rightFlipper.setPosition(Range.clip(rightFlipper.getPosition() + .005, 0, RIGHT_FLIPPER_DOWN));

                if(Math.abs(leftFlipper.getPosition() - LEFT_FLIPPER_DOWN) < .01) {
                    grabber.setPosition(GRABBER_OPEN);
                }

                flag = true;
            }

            //Right bumper controls grabber
            if(gamepad1.right_bumper) {
                grabber.setPosition(GRABBER_OPEN);
            }

            //Intakes
            if(gamepad1.left_bumper) { //Spit out glpyh
                leftIntake.setPower(-.95);
                rightIntake.setPower(-.95);
            } else if(gamepad1.left_trigger > .5) {  //Suck glyphs in
                leftIntake.setPower(.95);
                rightIntake.setPower(.95);
            } else {
                leftIntake.setPower(gamepad2.left_stick_y); //Joystick up sucks in glyphs
                rightIntake.setPower(gamepad2.right_stick_y);
            }

            if(gamepad1.b) led.setPower(1); //Red
            else if(gamepad1.x) led.setPower(-1); //Blue
            //No else, setPower(0) is located later

            //********** Gamepad 2 - Start + B ***********
            //Left bumper moves flipper to "flat" position - slightly up to jostle glyphs
            if(gamepad2.left_bumper && gamepad1.right_trigger < .5) { //Do not override gamepad1 flipper movement
                leftFlipper.setPosition(LEFT_FLIPPER_FLAT);
                rightFlipper.setPosition(RIGHT_FLIPPER_FLAT);
            }

            if(gamepad2.right_bumper && gamepad1.right_trigger < .5) {
                //Move lever medium speed until it nears the upward position, then move it slowly to be fully up
                if(Math.abs(glyphLever.getPosition() - GLYPH_LEVER_UP) < .20) glyphLever.setPosition(Range.clip(glyphLever.getPosition() - .001, 0, GLYPH_LEVER_UP));
                else glyphLever.setPosition(Range.clip(glyphLever.getPosition() - .01, 0, GLYPH_LEVER_UP));
            } else if(gamepad1.right_trigger < .5) glyphLever.setPosition(GLYPH_LEVER_DOWN);

            //Glyph lift controlled by triggers
            if(gamepad2.right_trigger > .1) glyphLift.setPower(gamepad2.right_trigger);
            else if(gamepad2.left_trigger > .1) glyphLift.setPower(-gamepad2.left_trigger);
            else glyphLift.setPower(0);

            //Move color sensor back to original position
            //Necessary if servo loses power in the middle of a match - sending new position restores power
            if(gamepad2.a) {
                colorSensorArm.setPosition(COLOR_SENSOR_ARM_INITIAL);
                colorSensorRotator.setPosition(COLOR_ROTATOR_INITIAL);
            }

            //Turn LED on if the glyph lever is up - allows driver to see position of lever
            //Otherwise, if gamepad1 buttons that control LEDs are not pressed, turn LED off
            if(Math.abs(glyphLever.getPosition() - GLYPH_LEVER_DOWN) > .2) led.setPower(-1);
            else if(!gamepad1.b && !gamepad1.x) led.setPower(0);

            executeThreads();

            //Add gamepad values to HashMap - Used for toggles

            //Telemetry statements
            telemetry.addData("Left Front Power", motorLeftFront.getPower());
            telemetry.addData("Left Back Power", motorLeftBack.getPower());
            telemetry.addData("Right Front Power", motorRightFront.getPower());
            telemetry.addData("Right Back Power", motorRightBack.getPower());
            telemetry.addData("Glyph Lever Position", glyphLever.getPosition());
            telemetry.addData("Glyph Lift Position", glyphLift.getCurrentPosition());
            telemetry.addData("Color Sensor Arm Position", colorSensorArm.getPosition());
            telemetry.addData("Color Sensor Rotator Position", colorSensorRotator.getPosition());
            telemetry.addData("Voltage: ", this.hardwareMap.voltageSensor.iterator().next().getVoltage());
            telemetry.update();
        }
    }

    private void executeThreads() {
        Iterator<TaskData> iterator = pendingTasks.iterator();
        while (iterator.hasNext()) {
            if(!opModeIsActive()) break;
            TaskData task = iterator.next();

            if(new Date().getTime() - task.startTime > task.delay) {
                task.task.runTask();
                iterator.remove();
            }
        }
    }
}