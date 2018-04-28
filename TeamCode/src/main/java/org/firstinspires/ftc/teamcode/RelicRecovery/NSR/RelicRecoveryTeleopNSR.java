package org.firstinspires.ftc.teamcode.RelicRecovery.NSR;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
 * This is the code for the teleop program for the robot built for the North Super Regional competition.
 *
 */
@Disabled
@TeleOp(name = "Relic Recovery Teleop NSR", group = "NSR")
public class RelicRecoveryTeleopNSR extends OpModeBase {
    private HashMap<String, Boolean> previousLoopValues = new HashMap<>();
    private double maxSpeed = 1;
    private ArrayList<TaskData> pendingTasks = new ArrayList<>();

    public void runOpMode() {
        super.runOpMode(OpModeType.TELEOP);
        boolean flag = true;

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

            if(gamepad1.right_trigger > .5) {
                if(flag) { //Will only run the first time that this conditional executes
                    flag = false;

                    pendingTasks.add(new TaskData(600, new ThreadTaskInterface() {
                        @Override
                        public void runTask() {
                            glyphLever.setPosition(GLYPH_LEVER_BACK);
                        }
                    }));
                }

                if(leftFlipper.getPosition() > (LEFT_FLIPPER_DOWN - .01) && leftFlipper.getPosition() < LEFT_FLIPPER_PARTIALLY_UP) { //- .01 used to correct for floating point errors
                    leftFlipper.setPosition(Range.clip(leftFlipper.getPosition() + .005, 0, 1));
                    rightFlipper.setPosition(Range.clip(rightFlipper.getPosition() - .005, 0, 1));
                } else {
                    leftFlipper.setPosition(LEFT_FLIPPER_UP);
                    rightFlipper.setPosition(RIGHT_FLIPPER_UP);
                }
            } else if (!gamepad1.right_bumper && !gamepad2.left_bumper) { //Do not override gamepad2 flipper movement
                leftFlipper.setPosition(Range.clip(leftFlipper.getPosition() - .005, LEFT_FLIPPER_DOWN, 1));
                rightFlipper.setPosition(Range.clip(rightFlipper.getPosition() + .005, 0, RIGHT_FLIPPER_DOWN));
                flag = true;
            }

            if(gamepad1.right_bumper && gamepad1.right_trigger < .5) {
                if(leftFlipper.getPosition() > (LEFT_FLIPPER_DOWN - .01) && leftFlipper.getPosition() < LEFT_FLIPPER_DOWN + .2) { //- .01 used to correct for floating point errors
                    leftFlipper.setPosition(Range.clip(leftFlipper.getPosition() + .005, 0, 1));
                    rightFlipper.setPosition(Range.clip(rightFlipper.getPosition() - .005, 0, 1));
                } else {
                    leftFlipper.setPosition(LEFT_FLIPPER_DOWN + .2);
                    rightFlipper.setPosition(RIGHT_FLIPPER_DOWN - .2);
                }
            }

            //Intakes
            if(gamepad1.left_bumper) {
                leftIntake.setPower(-.95);
                rightIntake.setPower(.95);
            } else if(gamepad1.left_trigger > .5) {  //Suck glyphs in
                leftIntake.setPower(.85);
                rightIntake.setPower(-.85);
            } else {
                leftIntake.setPower(Range.clip(gamepad2.left_stick_y, -.85, .85));
                rightIntake.setPower(Range.clip(-gamepad2.right_stick_y, -.85, .85));
            }

            if(gamepad1.b) led.setPower(1); //Red
            else if(gamepad1.x) led.setPower(-1); //Blue
            //No else, setPower(0) is located later

            //********** Gamepad 2 - Start + B ***********
            if(gamepad2.left_bumper && gamepad1.right_trigger < .5 && !gamepad1.right_bumper) { //Do not override gamepad1 flipper movement
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

            //Control color sensor arm
            if(gamepad2.a) {
                colorSensorArm.setPosition(COLOR_SENSOR_ARM_INITIAL_TELEOP);
                colorSensorRotator.setPosition(COLOR_ROTATOR_INITIAL);
            } else {
                if (gamepad2.dpad_up)
                    colorSensorArm.setPosition(Range.clip(colorSensorArm.getPosition() + .03, 0, 1));
                if (gamepad2.dpad_down)
                    colorSensorArm.setPosition(Range.clip(colorSensorArm.getPosition() - .03, 0, 1));
                if (gamepad2.dpad_left)
                    colorSensorRotator.setPosition(Range.clip(colorSensorRotator.getPosition() + .03, 0, 1));
                if (gamepad2.dpad_right)
                    colorSensorRotator.setPosition(Range.clip(colorSensorRotator.getPosition() - .03, 0, 1));
            }

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