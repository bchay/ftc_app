package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Servo Test", group = "Test Code")
public class ServoTest extends OpMode {
    private Servo leftGlyphGrabber;
    private Servo rightGlyphGrabber;
    private Servo arm;
    private Servo balancingStonePresser;
    private Servo relicGrabber;
    private Servo relicRotator;

    public void init() {
        leftGlyphGrabber = hardwareMap.servo.get("left grabber");
        rightGlyphGrabber = hardwareMap.servo.get("right grabber");
        arm = hardwareMap.servo.get("arm");
        balancingStonePresser = hardwareMap.servo.get("balancing stone presser");
        relicGrabber = hardwareMap.servo.get("relic grabber");
        relicRotator = hardwareMap.servo.get("relic rotator");
    }

    public void loop() {
        if(gamepad2.y) leftGlyphGrabber.setPosition(Range.clip(leftGlyphGrabber.getPosition() + .005, 0, 1));
        if(gamepad2.x) rightGlyphGrabber.setPosition(Range.clip(rightGlyphGrabber.getPosition() + .005, 0, 1));
        if(gamepad2.a) leftGlyphGrabber.setPosition(Range.clip(leftGlyphGrabber.getPosition() - .005, 0, 1));
        if(gamepad2.b) rightGlyphGrabber.setPosition(Range.clip(rightGlyphGrabber.getPosition() - .005, 0, 1));

        if(gamepad2.dpad_up) arm.setPosition(Range.clip(arm.getPosition() + .005, 0, 1));
        if(gamepad2.dpad_down) arm.setPosition(Range.clip(arm.getPosition() - .005, 0, 1));

        telemetry.addData("left glyph grabber", leftGlyphGrabber.getPosition());
        telemetry.addData("right glyph grabber", rightGlyphGrabber.getPosition());
        telemetry.addData("arm", arm.getPosition());
        telemetry.update();
    }
}
