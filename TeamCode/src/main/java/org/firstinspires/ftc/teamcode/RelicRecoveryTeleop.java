package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.Date;
import java.util.HashMap;
import java.util.Iterator;


/*
Gamepad Mappings:

Gamepad 1:
    Left Joystick: Control robot movement in XY plane - robot moves in direction of joystick
    Right Joystick: Left and right turns robot about Z axis

    Y: Toggle drivetrain reverse

    Right Trigger: Moves glyph stopper to up position. After 1 second delay, moves flipper up
        If not pressed, glyph flipper moves down. After 1 second, glyph stopper moves down as well.

    Left Bumper: Intake in
    Left Trigger: Intake out

    Right Button:  Set glyph flipper to partially up
Gamepad 2:
    Right Joystick: Right intake - Forward joystick brings glyph in
    Left Joystick: Left intake - Forward joystick brings glyph in

    Left Bumper: Move glyph flipper partially up - if not pressed, glyph flipper will move down
    Right Bumper: Move glyph lever towards intake - if not pressed, glyph lever will move towards flipper

    Right Trigger: Move Glyph Lift up
    Left Trigger: Move Glyph Lift down

    Y: Move intake down
    A: Move intake up
 */
@TeleOp(name = "Relic Recovery Teleop")
public class RelicRecoveryTeleop extends OpModeBase {
    private HashMap<String, Boolean> previousLoopValues = new HashMap<>();
    private boolean drivetrainReverse = false;
    private double maxSpeed = 1;

    private ArrayList<TaskData> pendingTasks = new ArrayList<>();

    public void runOpMode() {
        super.runOpMode(OpModeType.TELEOP);

        previousLoopValues.put("gamepad1.y", false);
        previousLoopValues.put("gamepad1.right_trigger", false);

        telemetry.addData("Ready to start program", "");
        telemetry.update();

        waitForStart();
        initializeServos(OpModeType.TELEOP); //Servos cannot be initialized during teleop init, only after start

        while (opModeIsActive()) {
            //********** Gamepad 1 - Start + A **********
            if(gamepad1.y && !previousLoopValues.get("gamepad1.y")) {
                drivetrainReverse = !drivetrainReverse;
            }

            if(gamepad1.x) maxSpeed = .5;
            else maxSpeed = 1;

            if(drivetrainReverse) {
                motorLeftFront.setPower(Range.clip(-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x, -maxSpeed, maxSpeed));
                motorLeftBack.setPower(Range.clip(-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x, -maxSpeed, maxSpeed));
                motorRightFront.setPower(Range.clip(-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x, -maxSpeed, maxSpeed));
                motorRightBack.setPower(Range.clip(-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x, -maxSpeed, maxSpeed));
            } else {
                motorLeftFront.setPower(Range.clip(gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x, -maxSpeed, maxSpeed));
                motorLeftBack.setPower(Range.clip(gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x, -maxSpeed, maxSpeed));
                motorRightFront.setPower(Range.clip(gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x, -maxSpeed, maxSpeed));
                motorRightBack.setPower(Range.clip(gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x, -maxSpeed, maxSpeed));
            }

            if(gamepad1.right_bumper && gamepad1.right_trigger < .5) {
                glyphFlipper.setPosition(GLYPH_FLIPPER_PARTIALLY_UP);
            } else if(!gamepad1.right_bumper && gamepad1.right_trigger < .5 && !gamepad2.left_bumper) {
                glyphFlipper.setPosition(GLYPH_FLIPPER_FLAT);
            }

            //Glyph flipper
            if(gamepad1.right_trigger > .5 && !previousLoopValues.get("gamepad1.right_trigger")) { //First time trigger is pressed
                glyphStopper.setPosition(GLYPH_STOPPER_UP); //Move stopper up

                //Add task
                pendingTasks.add(new TaskData(500, new ThreadTaskInterface() {
                    @Override
                    public void runTask() {
                        if(gamepad1.right_trigger > .5) {
                            glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
                        }
                    }
                }));

                //Glyph lever moves up after the flipper does to stop glyphs from becoming stuck underneath the flipper.
                pendingTasks.add(new TaskData(1300, new ThreadTaskInterface() {
                    @Override
                    public void runTask() {
                        //Stop glyph from getting stuck below glyph flipper
                        if(gamepad1.right_trigger > .5) glyphLever.setPosition(GLYPH_LEVER_DOWN_FLIPPER);
                    }
                }));
            } else if(gamepad1.right_trigger < .5 && previousLoopValues.get("gamepad1.right_trigger")) { //First time trigger is not pressed
                glyphLever.setPosition(GLYPH_LEVER_DOWN_INTAKE);

                pendingTasks.add(new TaskData(500, new ThreadTaskInterface() {
                    @Override
                    public void runTask() {
                        if(gamepad1.right_trigger < .5) glyphFlipper.setPosition(GLYPH_FLIPPER_FLAT);
                    }
                }));

                pendingTasks.add(new TaskData(900, new ThreadTaskInterface() {
                    @Override
                    public void runTask() {
                        if(gamepad1.right_trigger < .5) glyphStopper.setPosition(GLYPH_STOPPER_DOWN);
                    }
                }));
            }

            //Intakes
            if(gamepad1.left_bumper) { //Suck glyphs in
                leftIntake.setPower(-.95);
                rightIntake.setPower(.95);
            } else if(gamepad1.left_trigger > .5) {
                leftIntake.setPower(.85);
                rightIntake.setPower(-.85);
            } else {
                leftIntake.setPower(Range.clip(gamepad2.left_stick_y, -.85, .85));
                rightIntake.setPower(Range.clip(-gamepad2.right_stick_y, -.85, .85));
            }

            //********** Gamepad 2 - Start + B ***********

            if (gamepad2.left_bumper) {
                glyphFlipper.setPosition(GLYPH_FLIPPER_PARTIALLY_UP);
            } else if(gamepad1.right_trigger < .5 && !gamepad1.right_bumper) { //Do not override gamepad 1 flipper movement
                glyphFlipper.setPosition(Range.clip(glyphFlipper.getPosition() - .01, 0, GLYPH_FLIPPER_FLAT));
            }

            /*
            Move glyphLever up if gamepad1.right_bumper is pressed
            Else if statement stops lever from being automatically moved down if gamepad1.right_bumper sets position to GLYPH_LEVER_DOWN_FLIPPER
            Allow lever to be moved up and down repeatedly to eject glyph if it becomes stuck under flipper
            */
            if(gamepad2.right_bumper) glyphLever.setPosition(GLYPH_LEVER_UP);
            else if(Math.abs(glyphLever.getPosition() - GLYPH_LEVER_UP) < .1) glyphLever.setPosition(GLYPH_LEVER_DOWN_INTAKE);
            //else if(gamepad1.right_trigger < .5) glyphLever.setPosition(GLYPH_LEVER_DOWN_INTAKE);

            //Glyph lift controlled by triggers
            if(gamepad2.right_trigger > .1) glyphLift.setPower(-gamepad2.right_trigger);
            else if(gamepad2.left_trigger > .1) glyphLift.setPower(gamepad2.left_trigger);
            else glyphLift.setPower(0);

            //Y, A control moveIntake
            if(gamepad2.y) moveIntake.setPower(1); //Move intake up
            else if(gamepad2.a) moveIntake.setPower(-1); //Move intake down
            else moveIntake.setPower(0);

            //Add gamepad values to HashMap - Used for toggles
            previousLoopValues.put("gamepad1.y", gamepad1.y);
            previousLoopValues.put("gamepad1.right_trigger", (gamepad1.right_trigger > .5));

            //Control color sensor arm
            if(gamepad2.dpad_up) colorSensorArm.setPosition(Range.clip(colorSensorArm.getPosition() + .03, 0, 1));
            if(gamepad2.dpad_down) colorSensorArm.setPosition(Range.clip(colorSensorArm.getPosition() - .03, 0, 1));

            if(gamepad2.dpad_left) colorSensorRotator.setPosition(Range.clip(colorSensorRotator.getPosition() + .03, 0, 1));
            if(gamepad2.dpad_right) colorSensorRotator.setPosition(Range.clip(colorSensorRotator.getPosition() - .03, 0, 1));

            executeThreads();

            //Telemetry statements
            telemetry.addData("Glyph Lever Position", glyphLever.getPosition());
            telemetry.addData("Left Front Power", motorLeftFront.getPower());
            telemetry.addData("Left Back Power", motorLeftBack.getPower());
            telemetry.addData("Right Front Power", motorRightFront.getPower());
            telemetry.addData("Right Back Power", motorRightBack.getPower());
            telemetry.addData("Max speed - X", maxSpeed);
            telemetry.addData("Drivetrain direction - Y", drivetrainReverse ? "Reverse" : "Forward");
            telemetry.addData("Glyph Lift Position", glyphLift.getCurrentPosition());
            telemetry.addData("Color Sensor Arm", colorSensorArm.getPosition());
            telemetry.addData("Color Sensor Rotator", colorSensorRotator.getPosition());
            telemetry.addData("Voltage: ", this.hardwareMap.voltageSensor.iterator().next().getVoltage());
            telemetry.update();
        }

        colorSensorRotator.setPosition(COLOR_ROTATOR_INITIAL);
        sleep(500);
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