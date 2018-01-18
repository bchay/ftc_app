package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Servo Test", group = "Test Code")
public class ServoTest extends OpMode {
    private Servo colorSensorArm;
    private Servo colorSensorRotator;
    private Servo glyphFlipper;
    private Servo glyphMover;
    private Servo glyphStopper;

    public void init() {
        colorSensorArm = hardwareMap.servo.get("color arm");
        colorSensorRotator = hardwareMap.servo.get("color rotator");
        glyphFlipper = hardwareMap.servo.get("glyph flipper");
        glyphMover = hardwareMap.servo.get("glyph mover");
        glyphStopper = hardwareMap.servo.get("glyph stopper");
    }

    public void loop() {
        if(gamepad1.y) colorSensorArm.setPosition(Range.clip(colorSensorArm.getPosition() + .003, 0, 1));
        if(gamepad1.a) colorSensorArm.setPosition(Range.clip(colorSensorArm.getPosition() - .003, 0, 1));

        if(gamepad1.b) colorSensorRotator.setPosition(Range.clip(colorSensorRotator.getPosition() + .003, 0, 1));
        if(gamepad1.x) colorSensorRotator.setPosition(Range.clip(colorSensorRotator.getPosition() - .003, 0, 1));

        if(gamepad1.left_bumper) glyphFlipper.setPosition(Range.clip(glyphFlipper.getPosition() + .003, 0, 1));
        if(gamepad1.right_bumper) glyphFlipper.setPosition(Range.clip(glyphFlipper.getPosition() - .003, 0, 1));

        if(gamepad1.dpad_left) glyphMover.setPosition(Range.clip(glyphMover.getPosition() + .003, 0, 1));
        if(gamepad1.dpad_right) glyphMover.setPosition(Range.clip(glyphMover.getPosition() - .003, 0, 1));

        if(gamepad1.dpad_up) glyphStopper.setPosition(Range.clip(glyphStopper.getPosition() + .003, 0, 1));
        if(gamepad1.dpad_down) glyphStopper.setPosition(Range.clip(glyphStopper.getPosition() - .003, 0, 1));


        telemetry.addData("Arm Position", colorSensorArm.getPosition());
        telemetry.addData("Rotator Position", colorSensorRotator.getPosition());
        telemetry.addData("Glyph Stop Position", glyphStopper.getPosition());
        telemetry.update();
    }
}
