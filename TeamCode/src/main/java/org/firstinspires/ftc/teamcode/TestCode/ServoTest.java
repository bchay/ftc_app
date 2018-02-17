package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Servo Test", group = "Test Code")
public class ServoTest extends OpMode {
    private Servo colorSensorArm;
    private Servo colorSensorRotator;
    private Servo glyphFlipperLeft;
    private Servo glyphFlipperRight;
    private Servo glyphLever;

    public void init() {
        colorSensorArm = hardwareMap.servo.get("color arm");
        colorSensorRotator = hardwareMap.servo.get("color rotator");
        glyphFlipperLeft = hardwareMap.servo.get("glyph flipper left");
        glyphFlipperRight = hardwareMap.servo.get("glyph flipper right");
        glyphLever = hardwareMap.servo.get("glyph lever");
    }

    public void loop() {
        if(gamepad1.y) colorSensorArm.setPosition(Range.clip(colorSensorArm.getPosition() + .003, 0, 1));
        if(gamepad1.a) colorSensorArm.setPosition(Range.clip(colorSensorArm.getPosition() - .003, 0, 1));

        if(gamepad1.b) colorSensorRotator.setPosition(Range.clip(colorSensorRotator.getPosition() + .003, 0, 1));
        if(gamepad1.x) colorSensorRotator.setPosition(Range.clip(colorSensorRotator.getPosition() - .003, 0, 1));

        if(gamepad1.dpad_up) glyphFlipperLeft.setPosition(Range.clip(glyphFlipperLeft.getPosition() + .003, 0, 1));
        if(gamepad1.dpad_down) glyphFlipperLeft.setPosition(Range.clip(glyphFlipperLeft.getPosition() - .003, 0, 1));

        if(gamepad1.left_bumper) glyphFlipperRight.setPosition(Range.clip(glyphFlipperRight.getPosition() + .003, 0, 1));
        if(gamepad1.right_bumper) glyphFlipperRight.setPosition(Range.clip(glyphFlipperRight.getPosition() - .003, 0, 1));

        if(gamepad1.dpad_left) glyphLever.setPosition(Range.clip(glyphLever.getPosition() + .003, 0, 1));
        if(gamepad1.dpad_right) glyphLever.setPosition(Range.clip(glyphLever.getPosition() - .003, 0, 1));

        telemetry.addData("Jewel Arm - Y/A", colorSensorArm.getPosition());
        telemetry.addData("Rotator - B/X", colorSensorRotator.getPosition());
        telemetry.addData("Glyph Lever - dpad up / down", glyphLever.getPosition());
        telemetry.addData("Left Glyph Flipper - Left trigger / bumper", glyphFlipperLeft.getPosition());
        telemetry.addData("Right Glyph Flipper - Right trigger / bumper", glyphFlipperRight.getPosition());
        telemetry.update();
    }
}