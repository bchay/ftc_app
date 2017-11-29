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

    private HashMap<String, Boolean> previousLoopValues = new HashMap<String, Boolean>();

    public void runOpMode() {
        super.runOpMode(RelicRecoveryTeleop.class);

        telemetry.addData("Ready to start program", "");
        telemetry.update();

        waitForStart();
        initializeServos(); //Servos cannot be initialized during teleop init, only after start

        telemetry.addData("Loop", "");
        telemetry.update();

        while(opModeIsActive()) {
            motorLeft1.setPower(Math.pow(gamepad1.left_stick_y, 3) * (slowMode ? .5 : 1));
            motorLeft2.setPower(Math.pow(gamepad1.left_stick_y, 3) * (slowMode ? .5 : 1));
            motorRight1.setPower(Math.pow(gamepad1.right_stick_y, 3) * (slowMode ? .5 : 1));
            motorRight2.setPower(Math.pow(gamepad1.right_stick_y, 3) * (slowMode ? .5 : 1));

            if(glyphLift.getCurrentPosition() < 3000 && gamepad2.left_stick_y > 0) { //If lift is moving down, close servos; gamepad joystick y returns positive value when moving down
                leftGlyphGrabber.setPosition(LEFT_GLYPH_GRABBR_CLOSED);
                rightGlyphGrabber.setPosition(RIGHT_GLYPH_GRABBR_CLOSED);
            } else if(gamepad2.x && !previousLoopValues.get("gamepad2.x")) { //X button has just been pressed - toggle glyph grabbers
                    //Need to check distance rather than == because floating point error may cause position to not be equal to constant
                    if(Math.abs(leftGlyphGrabber.getPosition() - LEFT_GLYPH_GRABBR_CLOSED) < .1) leftGlyphGrabber.setPosition(LEFT_GLYPH_GRABBR_OPEN);
                    else leftGlyphGrabber.setPosition(LEFT_GLYPH_GRABBR_CLOSED);

                    if(Math.abs(rightGlyphGrabber.getPosition() - RIGHT_GLYPH_GRABBR_CLOSED) < .1) rightGlyphGrabber.setPosition(RIGHT_GLYPH_GRABBR_OPEN);
                    else rightGlyphGrabber.setPosition(RIGHT_GLYPH_GRABBR_CLOSED);
                }
            }

            if(gamepad1.b && !previousLoopValues.get("gamepad1.b")) { //B button has just been pressed - toggle slow mode
                slowMode = !slowMode;
            }

            previousLoopValues.put("gamepad2.x", gamepad2.x);
            previousLoopValues.put("gamepad1.b", gamepad1.b);

            if(gamepad2.dpad_up) relicGrabber.setPosition(Range.clip(relicGrabber.getPosition() + .01, 0, 1));
            if(gamepad2.dpad_down) relicGrabber.setPosition(Range.clip(relicGrabber.getPosition() - .01, 0, 1));
            if(gamepad2.dpad_left) relicRotator.setPosition(Range.clip(relicRotator.getPosition() + .01, 0, 1));
            if(gamepad2.dpad_right) relicRotator.setPosition(Range.clip(relicRotator.getPosition() - .01, 0, 1));

            glyphLift.setPower(gamepad2.left_stick_y);
            relicLift.setPower(gamepad2.right_stick_y);

            telemetry.addData("Left Power", motorLeft1.getPower());
            telemetry.addData("Right Power", motorRight1.getPower());
            telemetry.addData("Slow Mode", slowMode);
            telemetry.addData("Left Grabber Position", leftGlyphGrabber.getPosition());
            telemetry.addData("Right Grabber Position", rightGlyphGrabber.getPosition());
            telemetry.addData("Lift Current Position", glyphLift.getCurrentPosition());
            telemetry.addData("Voltage: ", this.hardwareMap.voltageSensor.iterator().next().getVoltage());
            telemetry.update();
        }
}