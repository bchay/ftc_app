package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import java.util.HashMap;


/*
Gamepad Mappings:

Gamepad 1:
    Left Joystick: Left Wheels Power
    Right Joystick: Right Wheels Power

    B: Toggle Slow Mode
    X: Toggle drivetrain reverse

Gamepad 2:
    X: Toggle Glyph Grabbers

    Dpad Up: Relic Grabber - Increase Position
    Dpad Down: Relic Grabber - Decrease Position
    Dpad Left: Relic Rotator - Increase Position
    Dpad Right: Relic Rotator - Decrease Position

    Left Joystick: Glyph Lift
    Left Joystick: Relic Lift
 */

@TeleOp(name = "Relic Recovery Teleop")
public class RelicRecoveryTeleop extends OpModeBase {
    private boolean slowMode = false;
    private boolean drivetrainReverse = false;

    private HashMap<String, Boolean> previousLoopValues = new HashMap<String, Boolean>();

    public void runOpMode() {
        super.runOpMode(RelicRecoveryTeleop.class);

        telemetry.addData("Ready to start program", "");
        telemetry.update();

        waitForStart();
        initializeServos(); //Servos cannot be initialized during teleop init, only after start

        while (opModeIsActive()) {

            if(!drivetrainReverse) {
                motorLeft1.setPower(Math.pow(gamepad1.left_stick_y, 3) * (slowMode ? .5 : 1));
                motorLeft2.setPower(Math.pow(gamepad1.left_stick_y, 3) * (slowMode ? .5 : 1));
                motorRight1.setPower(Math.pow(gamepad1.right_stick_y, 3) * (slowMode ? .5 : 1));
                motorRight2.setPower(Math.pow(gamepad1.right_stick_y, 3) * (slowMode ? .5 : 1));
            } else {
                motorLeft1.setPower(Math.pow(-gamepad1.right_stick_y, 3) * (slowMode ? .5 : 1));
                motorLeft2.setPower(Math.pow(-gamepad1.right_stick_y, 3) * (slowMode ? .5 : 1));
                motorRight1.setPower(Math.pow(-gamepad1.left_stick_y, 3) * (slowMode ? .5 : 1));
                motorRight2.setPower(Math.pow(-gamepad1.left_stick_y, 3) * (slowMode ? .5 : 1));
            }

            //If lift is moving down, close servos; gamepad joystick y returns positive value when moving down
            //If it is fully closed already, do not move it to the slightly open position
            if ((glyphLift.getCurrentPosition() < 3000 && gamepad2.left_stick_y > 0) && (Math.abs(leftGlyphGrabber.getPosition() - LEFT_GLYPH_GRABBR_OPEN) < .1)) {
                leftGlyphGrabber.setPosition(LEFT_GLYPH_GRABBR_CLOSED - .3); //Slightly further apart than fully closed
                rightGlyphGrabber.setPosition(RIGHT_GLYPH_GRABBR_CLOSED + .15);
            } else if (gamepad2.x && !previousLoopValues.get("gamepad2.x")) { //X button has just been pressed - toggle glyph grabbers
                //Need to check distance rather than == because floating point error may cause position to not be equal to constant
                if (Math.abs(leftGlyphGrabber.getPosition() - LEFT_GLYPH_GRABBR_CLOSED) < .1)
                    leftGlyphGrabber.setPosition(LEFT_GLYPH_GRABBR_OPEN);
                else leftGlyphGrabber.setPosition(LEFT_GLYPH_GRABBR_CLOSED);

                if (Math.abs(rightGlyphGrabber.getPosition() - RIGHT_GLYPH_GRABBR_CLOSED) < .1)
                    rightGlyphGrabber.setPosition(RIGHT_GLYPH_GRABBR_OPEN);
                else rightGlyphGrabber.setPosition(RIGHT_GLYPH_GRABBR_CLOSED);
            }

            if(gamepad1.b && !previousLoopValues.get("gamepad1.b")) { //B button has just been pressed - toggle slow mode
                slowMode = !slowMode;
            }

            if(gamepad1.x && !previousLoopValues.get("gamepad1.x")) { //B button has just been pressed - toggle slow mode
                drivetrainReverse = !drivetrainReverse;
            }

            previousLoopValues.put("gamepad2.x", gamepad2.x);
            previousLoopValues.put("gamepad1.b", gamepad1.b);
            previousLoopValues.put("gamepad1.x", gamepad1.x);

            if(gamepad2.dpad_left) relicGrabber.setPosition(Range.clip(relicGrabber.getPosition() - .01, 0, 1));
            if(gamepad2.dpad_right) relicGrabber.setPosition(Range.clip(relicGrabber.getPosition() + .01, 0, 1));

            if(gamepad2.right_bumper) {
                relicRotator.setPosition(RELIC_ROTATOR_PARALLEL);
            } else {
                if (gamepad2.dpad_up) relicRotator.setPosition(Range.clip(relicRotator.getPosition() + .01, 0, 1));
                if (gamepad2.dpad_down) relicRotator.setPosition(Range.clip(relicRotator.getPosition() - .01, 0, 1));
            }

            glyphLift.setPower(gamepad2.left_stick_y);
            relicLift.setPower(Range.clip(gamepad2.right_stick_y, -.5, .5));

            telemetry.addData("Left Power", motorLeft1.getPower());
            telemetry.addData("Right Power", motorRight1.getPower());
            telemetry.addData("Slow Mode", slowMode);
            telemetry.addData("Left Grabber Position", leftGlyphGrabber.getPosition());
            telemetry.addData("Right Grabber Position", rightGlyphGrabber.getPosition());
            telemetry.addData("Relic Rotator Position", relicRotator.getPosition());
            telemetry.addData("Relic Grabber Position", relicGrabber.getPosition());
            telemetry.addData("Lift Current Position", glyphLift.getCurrentPosition());
            telemetry.addData("Voltage: ", this.hardwareMap.voltageSensor.iterator().next().getVoltage());
            telemetry.update();
        }
    }
}