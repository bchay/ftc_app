package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import java.util.HashMap;


/*
Gamepad Mappings:

Gamepad 1:
    Two drivetrain modes: Standard Mecanum and Hybrid Mecanum and Tank
        Standard Mecanum:
            Left Joystick: Control robot movement in XY plane
            Right Joystick: Left and right turns robot about Z axis

        Hybrid:

            Both joysticks in one direction - Move robot in that direction
            One joystick up and one down - Turn robot in the direction of the lower joystick (Tank drive style turning)

    B: Toggle drive modes - Mecanum and Hybrid
    Y: Toggle drivetrain reverse
    X: Toggle slow mode

    Right Trigger: Move flipper up, set glyph stop to down position
        If not pressed, glyph flipper moves down. Once it is down, glyph stopper moves down as well.

    Left Bumper: Intake in
    Left Trigger: Intake out

    Right Button: Activate slow motion mode, set glyph stopper servo to up
Gamepad 2:
    Right Joystick: Right intake - Forward joystick brings glyph in
    Left Joystick: Left intake - Forward joystick brings glyph in

    Left Bumper: Move glyph flipper up - if not pressed, glyph flipper will move down
    Right Bumper: Move glyph lever up - if not pressed, glyph lever will move down

    Right Trigger: Move Glyph Lift up
    Left Trigger: Move Glyph Lift down

    Y: Move intake down
    A: Move intake up
 */

@TeleOp(name = "Relic Recovery Teleop")
public class RelicRecoveryTeleop extends OpModeBase {
    private HashMap<String, Boolean> previousLoopValues = new HashMap<>();

    private boolean driveModeMecanum = true;
    private boolean drivetrainReverse = false;
    private double maxSpeed = 1;

    public void runOpMode() {
        super.runOpMode(RelicRecoveryTeleop.class);

        previousLoopValues.put("gamepad1.b", false);
        previousLoopValues.put("gamepad1.y", false);
        previousLoopValues.put("gamepad1.x", false);
        previousLoopValues.put("gamepad1.right_trigger", false);

        telemetry.addData("Ready to start program", "");
        telemetry.update();

        waitForStart();
        initializeServos(RelicRecoveryTeleop.class); //Servos cannot be initialized during teleop init, only after start

        while (opModeIsActive()) {
            //********** Gamepad 1 - Start + A **********

            if(gamepad1.b && !previousLoopValues.get("gamepad1.b")) {
                driveModeMecanum = !driveModeMecanum;
            }

            if(gamepad1.y && !previousLoopValues.get("gamepad1.y")) {
                drivetrainReverse = !drivetrainReverse;
            }

            /*if(gamepad1.x && !previousLoopValues.get("gamepad1.x")) {
                if(maxSpeed == 1) maxSpeed = .5; //Toggle the speed between fast and slow
                else maxSpeed = 1;
            }*/

            if(gamepad1.right_bumper) {
                maxSpeed = .5;
                glyphStopper.setPosition(GLYPH_STOPPER_UP);
            } else {
                maxSpeed = 1;
                if(Math.abs(glyphFlipper.getPosition() - GLYPH_FLIPPER_FLAT) < .05) glyphStopper.setPosition(GLYPH_STOPPER_DOWN);

            }

            if(driveModeMecanum) {
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
            } else { //Custom drive mode - hybrid between tank and mecanum
                //Both gamepad joystick values are close = move robot in direction
                if((Math.abs(gamepad1.left_stick_y - gamepad1.right_stick_y) < .4) && (Math.abs(gamepad1.left_stick_x - gamepad1.right_stick_x) < .4)) {
                    if(drivetrainReverse) {
                        motorLeftFront.setPower(Range.clip(-gamepad1.left_stick_y - gamepad1.left_stick_x, -maxSpeed, maxSpeed));
                        motorLeftBack.setPower(Range.clip(-gamepad1.left_stick_y + gamepad1.left_stick_x, -maxSpeed, maxSpeed));
                        motorRightFront.setPower(Range.clip(-gamepad1.left_stick_y + gamepad1.left_stick_x, -maxSpeed, maxSpeed));
                        motorRightBack.setPower(Range.clip(-gamepad1.left_stick_y - gamepad1.left_stick_x, -maxSpeed, maxSpeed));
                    } else {
                        motorLeftFront.setPower(Range.clip(gamepad1.left_stick_y + gamepad1.left_stick_x, -maxSpeed, maxSpeed));
                        motorLeftBack.setPower(Range.clip(gamepad1.left_stick_y - gamepad1.left_stick_x, -maxSpeed, maxSpeed));
                        motorRightFront.setPower(Range.clip(gamepad1.left_stick_y - gamepad1.left_stick_x, -maxSpeed, maxSpeed));
                        motorRightBack.setPower(Range.clip(gamepad1.left_stick_y + gamepad1.left_stick_x, -maxSpeed, maxSpeed));
                    }
                } else if((Math.abs(gamepad1.left_stick_y - gamepad1.right_stick_y) > 1.6) && (Math.abs(gamepad1.left_stick_x - gamepad1.right_stick_x) < .4)) {
                    //One joystick is pointing up, and the other is pointing down = turn robot
                    //Motor powers are the same even if drivetrain is reversed
                    motorLeftFront.setPower(Range.clip(-gamepad1.left_stick_y, -maxSpeed, maxSpeed));
                    motorLeftBack.setPower(Range.clip(-gamepad1.left_stick_y, -maxSpeed, maxSpeed));
                    motorRightFront.setPower(Range.clip(gamepad1.left_stick_y, -maxSpeed, maxSpeed));
                    motorRightBack.setPower(Range.clip(gamepad1.left_stick_y, -maxSpeed, maxSpeed));
                } else {
                    motorLeftFront.setPower(0);
                    motorLeftBack.setPower(0);
                    motorRightFront.setPower(0);
                    motorRightBack.setPower(0);
                }
            }

            /*
            //Control Glyph Flipper
            if(gamepad1.right_trigger > .5 && !previousLoopValues.get("gamepad1.right_trigger")) {
                new Thread(new Runnable() {
                    @Override
                    public void run() {
                        glyphStopper.setPosition(GLYPH_STOPPER_UP); //Move stopper up
                        sleep(1000);
                        if(opModeIsActive()) glyphFlipper.setPosition(GLYPH_FLIPPER_VERTICAL);
                    }
                }).start();
            } else if(gamepad1.right_trigger < .5 && previousLoopValues.get("gamepad1.right_trigger")) { //When trigger is not pressed, but it was pressed previously
                new Thread(new Runnable() {
                    @Override
                    public void run() {
                        glyphFlipper.setPosition(GLYPH_FLIPPER_FLAT);
                        sleep(1000);
                        if(opModeIsActive()) glyphStopper.setPosition(GLYPH_STOPPER_DOWN); //Move stopper up
                    }
                }).start();
            }
            */

            if(gamepad1.right_trigger > .5) { //Move flipper up
                glyphStopper.setPosition(Range.clip(glyphStopper.getPosition() - .05, GLYPH_STOPPER_UP, GLYPH_STOPPER_DOWN)); //Move stopper up

                //Only move glyph flipper up if the stopper is already up
                //Using setPosition(getPosition()) so that servo moves slower than simply setPosition(CONSTANT)
                if(Math.abs(glyphStopper.getPosition() - GLYPH_STOPPER_UP) < .05) glyphFlipper.setPosition(Range.clip(glyphFlipper.getPosition() + .02, GLYPH_FLIPPER_FLAT, GLYPH_FLIPPER_VERTICAL));
            } else {
                glyphFlipper.setPosition(Range.clip(glyphFlipper.getPosition() - .02, GLYPH_FLIPPER_FLAT, GLYPH_FLIPPER_VERTICAL));

                //Only move stopper down if the glyph flipper is already down
                //Using setPosition(getPosition()) so that servo moves slower than simply setPosition(CONSTANT)
                if(Math.abs(glyphFlipper.getPosition() - GLYPH_FLIPPER_FLAT) < .1) glyphStopper.setPosition(Range.clip(glyphStopper.getPosition() + .01, 0, GLYPH_STOPPER_DOWN));
            }

            //Intakes
            if(gamepad1.left_trigger > .5) { //Suck glyphs in
                leftIntake.setPower(-.65);
                rightIntake.setPower(.65);
            } else if(gamepad1.left_bumper) {
                leftIntake.setPower(.65);
                rightIntake.setPower(-.65);
            } else {
                leftIntake.setPower(Range.clip(-gamepad2.left_stick_y, -.65, .65));
                rightIntake.setPower(Range.clip(gamepad2.right_stick_y, -.65, .65));
            }




            //********** Gamepad 2 - Start + B ***********

            //Left Bumper: Move glyph flipper up - if not pressed, glyph flipper will move down
            //Right Bumper: Move glyph lever up - if not pressed, glyph lever will move down

            if (gamepad2.left_bumper) {
                glyphFlipper.setPosition(GLYPH_FLIPPER_PARTIALLY_UP);
            } else if(gamepad1.right_trigger < .5 && Math.abs(glyphStopper.getPosition() - GLYPH_STOPPER_DOWN) < .05) { //Do not override gamepad1 flipper movement
                glyphFlipper.setPosition(GLYPH_FLIPPER_FLAT);
            }

            if (gamepad2.right_bumper) {
                glyphLever.setPosition(GLYPH_LEVER_UP);
            } else glyphLever.setPosition(GLYPH_LEVER_DOWN);


            //Glyph lift controlled by triggers
            if(gamepad2.right_trigger > .1) glyphLift.setPower(-gamepad2.right_trigger);
            else if(gamepad2.left_trigger > .1) glyphLift.setPower(gamepad2.left_trigger);
            else glyphLift.setPower(0);

            //Y, A control moveIntake
            if(gamepad2.y) moveIntake.setPower(1);
            else if(gamepad2.a) moveIntake.setPower(-1);
            else moveIntake.setPower(0);

            //Add gamepad values to HashMap - Used for toggles
            previousLoopValues.put("gamepad1.b", gamepad1.b);
            previousLoopValues.put("gamepad1.y", gamepad1.y);
            previousLoopValues.put("gamepad1.x", gamepad1.x);
            //previousLoopValues.put("gamepad1.right_trigger", (gamepad1.right_trigger > .5));


            //Telemetry statements
            telemetry.addData("Glyph Lever Position", glyphLever.getPosition());
            telemetry.addData("Left Front Power", motorLeftFront.getPower());
            telemetry.addData("Left Back Power", motorLeftBack.getPower());
            telemetry.addData("Right Front Power", motorRightFront.getPower());
            telemetry.addData("Right Back Power", motorRightBack.getPower());
            telemetry.addData("Max speed - X", maxSpeed);
            telemetry.addData("Drivetrain direction - Y", drivetrainReverse ? "Reverse" : "Forward");
            telemetry.addData("Drive Mode - B", driveModeMecanum ? "Mecanum" : "Custom");
            telemetry.addData("Glyph Lift Position", glyphLift.getCurrentPosition());
            telemetry.addData("Voltage: ", this.hardwareMap.voltageSensor.iterator().next().getVoltage());
            telemetry.update();
        }
    }
}