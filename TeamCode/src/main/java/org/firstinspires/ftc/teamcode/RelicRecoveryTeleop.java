package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.HashMap;


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
    private HashMap<String, Object> previousLoopValues = new HashMap<>();
    private boolean drivetrainReverse = false;
    private double maxSpeed = 1;
    private boolean runIntake = true;

    private ArrayList<TaskData> pendingTasks = new ArrayList<>();

    public void runOpMode() {
        super.runOpMode(RelicRecoveryTeleop.class);

        //Add initial values to HashMap to avoid NullPointerException during first loop iteration
        previousLoopValues.put("gamepad1.y", false);
        previousLoopValues.put("left intake encoder", 0);
        previousLoopValues.put("right intake encoder", 0);
        previousLoopValues.put("left intake power", 0.0); //Default value needs to be a double to avoid ClassCastException
        previousLoopValues.put("right intake power", 0.0);

        telemetry.addData("Ready to start program", "");
        telemetry.update();

        waitForStart();
        initializeServos(RelicRecoveryTeleop.class); //Servos cannot be initialized during teleop init, only after start

        while (opModeIsActive()) {
            //********** Gamepad 1 - Start + A **********
            if(gamepad1.y && !(Boolean) previousLoopValues.get("gamepad1.y")) {
                drivetrainReverse = !drivetrainReverse;
            }

            if(gamepad1.x) maxSpeed = .5;
            else maxSpeed = 1;

            if(drivetrainReverse) {
                motorLeftFront.setPower(Range.clip(-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x, -maxSpeed, maxSpeed));
                motorLeftBack.setPower(Range.clip(-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x, -maxSpeed, maxSpeed));
                motorRightFront.setPower(Range.clip(-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x, -maxSpeed, maxSpeed));
                motorRightBack.setPower(Range.clip(-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x, -maxSpeed, maxSpeed));
            } else {
                motorLeftFront.setPower(Range.clip(gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x, -maxSpeed, maxSpeed));
                motorLeftBack.setPower(Range.clip(gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x, -maxSpeed, maxSpeed));
                motorRightFront.setPower(Range.clip(gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x, -maxSpeed, maxSpeed));
                motorRightBack.setPower(Range.clip(gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x, -maxSpeed, maxSpeed));
            }

            if(gamepad1.right_bumper && gamepad1.right_trigger < .5) {
                glyphFlipperLeft.setPosition(GLYPH_FLIPPER_PARTIALLY_UP_LEFT);
                glyphFlipperRight.setPosition(GLYPH_FLIPPER_PARTIALLY_UP_RIGHT);
            } else if(!gamepad1.right_bumper && gamepad1.right_trigger < .5 && !gamepad2.left_bumper) {
                glyphFlipperLeft.setPosition(GLYPH_FLIPPER_FLAT_LEFT);
                glyphFlipperRight.setPosition(GLYPH_FLIPPER_FLAT_RIGHT);
            }

            //Intakes
//            if(runIntake && ((Math.abs(leftIntake.getCurrentPosition() - (int) previousLoopValues.get("left intake encoder")) > 0) && (Math.abs(rightIntake.getCurrentPosition() - (int) previousLoopValues.get("right intake encoder")) > 0)) || (leftIntake.getPower() == 0 || rightIntake.getPower() == 0) || ((((double) previousLoopValues.get("left intake power")) == 0) || (((double) previousLoopValues.get("right intake power")) == 0))) {
                if (gamepad1.left_bumper) { //Suck glyphs in
                    leftIntake.setPower(1);
                    rightIntake.setPower(1);
                } else if (gamepad1.left_trigger > .5) {
                    leftIntake.setPower(-1);
                    rightIntake.setPower(-1);
                } else {
                    leftIntake.setPower(-gamepad2.left_stick_y);
                    rightIntake.setPower(-gamepad2.right_stick_y);
                }
//            } else if(runIntake) { //Glyph is currently stuck in the intake, the intake wheels are not spinning
//                leftIntake.setPower(-.4); //Reverse one side of intake so that glyph aligns properly
//                rightIntake.setPower(1);
//
//                runIntake = false;
//
//                pendingTasks.add(new TaskData(400, new ThreadTaskInterface() {
//                    @Override
//                    public void runTask() {
//                        runIntake = true;
//                    }
//                }));
//            }

            //********** Gamepad 2 - Start + B ***********

            if (gamepad2.left_bumper) {
                glyphFlipperLeft.setPosition(GLYPH_FLIPPER_PARTIALLY_UP_LEFT);
                glyphFlipperRight.setPosition(GLYPH_FLIPPER_PARTIALLY_UP_RIGHT);
            } else if(gamepad1.right_trigger < .5 && !gamepad1.right_bumper) { //Do not override gamepad 1 flipper movement
                glyphFlipperLeft.setPosition(Range.clip(glyphFlipperLeft.getPosition() - .01, 0, GLYPH_FLIPPER_FLAT_LEFT));
                glyphFlipperRight.setPosition(Range.clip(glyphFlipperRight.getPosition() - .01, 0, GLYPH_FLIPPER_FLAT_RIGHT));
            }

            if(gamepad1.right_trigger > .5) {
                glyphFlipperLeft.setPosition(GLYPH_FLIPPER_VERTICAL_LEFT);
                glyphFlipperRight.setPosition(GLYPH_FLIPPER_VERTICAL_RIGHT);
            } else {
                glyphFlipperLeft.setPosition(GLYPH_FLIPPER_VERTICAL_LEFT);
                glyphFlipperRight.setPosition(GLYPH_FLIPPER_VERTICAL_RIGHT);
            }



            /*
            Move glyphLever up if gamepad1.right_bumper is pressed
            Else if statement stops lever from being automatically moved down if gamepad1.right_bumper sets position to GLYPH_LEVER_DOWN_FLIPPER
            Allow lever to be moved up and down repeatedly to eject glyph if it becomes stuck under flipper
            */
            if(gamepad2.right_bumper) glyphLever.setPosition(GLYPH_LEVER_UP);
            else if(Math.abs(glyphLever.getPosition() - GLYPH_LEVER_UP) < .1) glyphLever.setPosition(GLYPH_LEVER_DOWN);

            //Glyph lift controlled by triggers
            if(gamepad2.right_trigger > .1) glyphLift.setPower(-gamepad2.right_trigger);
            else if(gamepad2.left_trigger > .1) glyphLift.setPower(gamepad2.left_trigger);
            else glyphLift.setPower(0);

            //Y, A control moveIntake
            if(gamepad2.y) moveIntake.setPower(1); //Move intake up
            else if(gamepad2.a) moveIntake.setPower(-1); //Move intake down
            else moveIntake.setPower(0);

            //Control color sensor arm
            if(gamepad2.dpad_up) colorSensorArm.setPosition(Range.clip(colorSensorArm.getPosition() + .03, 0, 1));
            if(gamepad2.dpad_down) colorSensorArm.setPosition(Range.clip(colorSensorArm.getPosition() - .03, 0, 1));

            if(gamepad2.dpad_left) colorSensorRotator.setPosition(Range.clip(colorSensorRotator.getPosition() + .03, 0, 1));
            if(gamepad2.dpad_right) colorSensorRotator.setPosition(Range.clip(colorSensorRotator.getPosition() - .03, 0, 1));

            TaskData.executeThreads(pendingTasks, this);

            //Telemetry statements
            telemetry.addData("Left Front Power", motorLeftFront.getPower());
            telemetry.addData("Left Back Power", motorLeftBack.getPower());
            telemetry.addData("Right Front Power", motorRightFront.getPower());
            telemetry.addData("Right Back Power", motorRightBack.getPower());

            telemetry.addData("Max speed - X", maxSpeed);
            telemetry.addData("Drivetrain direction - Y", drivetrainReverse ? "Reverse" : "Forward");

            telemetry.addData("Glyph Lift Position", glyphLift.getCurrentPosition());
            telemetry.addData("Glyph Lever Position", glyphLever.getPosition());
            telemetry.addData("Color Sensor Arm", colorSensorArm.getPosition());
            telemetry.addData("Color Sensor Rotator", colorSensorRotator.getPosition());

            telemetry.addData("Left intake encoder delta", Math.abs(leftIntake.getCurrentPosition() - (int) previousLoopValues.get("left intake encoder")));
            telemetry.addData("Right intake encoder delta", Math.abs(rightIntake.getCurrentPosition() - (int) previousLoopValues.get("right intake encoder")));

            telemetry.addData("Voltage: ", this.hardwareMap.voltageSensor.iterator().next().getVoltage());
            telemetry.update();


            //Add gamepad values to HashMap - Used for toggles
            previousLoopValues.put("gamepad1.y", gamepad1.y);
            previousLoopValues.put("left intake encoder", leftIntake.getCurrentPosition());
            previousLoopValues.put("right intake encoder", rightIntake.getCurrentPosition());
            previousLoopValues.put("left intake power", leftIntake.getPower());
            previousLoopValues.put("right intake power", rightIntake.getPower());
        }
    }
}