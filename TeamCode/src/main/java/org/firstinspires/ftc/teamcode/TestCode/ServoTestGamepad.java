package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Servo Test Gamepad", group = "Test Code")
public class ServoTestGamepad extends OpMode {
    Servo s1;
    Servo s2;
    Servo s3;
    Servo s4;
    Servo s5;
    Servo s6;
    Servo s7;
    Servo s8;
    Servo s9;
    Servo s10;
    Servo s11;
    Servo s12;



    public void init() {
        s1 = hardwareMap.servo.get("1"); //gamepad 1 dpad left/right - Jewel arm .141
        s2 = hardwareMap.servo.get("2");
        s3 = hardwareMap.servo.get("3"); //A, Y - Jewel rotator - .082
        s4 = hardwareMap.servo.get("4");
        s5 = hardwareMap.servo.get("5"); //Grabber 1.right trigger
        s6 = hardwareMap.servo.get("6");
        s7 = hardwareMap.servo.get("7");
        s8 = hardwareMap.servo.get("8");
        s9 = hardwareMap.servo.get("9"); //Right flipper, gamepad2 a, y
        s10 = hardwareMap.servo.get("10"); //Glyph lever - .931
        s11 = hardwareMap.servo.get("11"); //NEW LEFT FLIPPER
        s12 = hardwareMap.servo.get("12");
    }

    public void loop() {
        if(gamepad1.dpad_left) s1.setPosition(Range.clip(s1.getPosition() + .001, 0, 1));
        else if(gamepad1.dpad_right) s1.setPosition(Range.clip(s1.getPosition() - .001, 0, 1));

        if(gamepad1.dpad_up) s2.setPosition(Range.clip(s2.getPosition() + .001, 0, 1));
        else if(gamepad1.dpad_down) s2.setPosition(Range.clip(s2.getPosition() - .001, 0, 1));

        if(gamepad1.a) s3.setPosition(Range.clip(s3.getPosition() + .001, 0, 1));
        else if(gamepad1.y) s3.setPosition(Range.clip(s3.getPosition() - .001, 0, 1));

        if(gamepad1.x) s4.setPosition(Range.clip(s4.getPosition() + .001, 0, 1));
        else if(gamepad1.b) s4.setPosition(Range.clip(s4.getPosition() - .001, 0, 1));

        if(gamepad1.right_trigger > .5) s5.setPosition(Range.clip(s5.getPosition() + .001, 0, 1));
        else if(gamepad1.left_trigger > .5) s5.setPosition(Range.clip(s5.getPosition() - .001, 0, 1));

        if(gamepad1.right_bumper) s6.setPosition(Range.clip(s6.getPosition() + .001, 0, 1));
        else if(gamepad1.left_bumper) s6.setPosition(Range.clip(s6.getPosition() - .001, 0, 1));

        if(gamepad2.dpad_left) s7.setPosition(Range.clip(s7.getPosition() + .001, 0, 1));
        else if(gamepad2.dpad_right) s7.setPosition(Range.clip(s7.getPosition() - .001, 0, 1));

        if(gamepad2.dpad_up) s8.setPosition(Range.clip(s8.getPosition() + .001, 0, 1));
        else if(gamepad2.dpad_down) s8.setPosition(Range.clip(s8.getPosition() - .001, 0, 1));

        if(gamepad2.a) s9.setPosition(Range.clip(s9.getPosition() + .001, 0, 1));
        else if(gamepad2.y) s9.setPosition(Range.clip(s9.getPosition() - .001, 0, 1));

        if(gamepad2.x) s10.setPosition(Range.clip(s10.getPosition() + .001, 0, 1));
        else if(gamepad2.b) s10.setPosition(Range.clip(s10.getPosition() - .001, 0, 1));

        if(gamepad2.right_bumper) s11.setPosition(Range.clip(s11.getPosition() + .001, 0, 1));
        else if(gamepad2.left_bumper) s11.setPosition(Range.clip(s11.getPosition() - .001, 0, 1));

        if(gamepad2.left_trigger > .5) s12.setPosition(Range.clip(s12.getPosition() + .001, 0, 1));
        else if(gamepad2.right_trigger > .5) s12.setPosition(Range.clip(s12.getPosition() - .001, 0, 1));

        telemetry.addData("s1", s1.getPosition());
        telemetry.addData("s2", s2.getPosition());
        telemetry.addData("s3", s3.getPosition());
        telemetry.addData("s4", s4.getPosition());
        telemetry.addData("s5", s5.getPosition());
        telemetry.addData("s6", s6.getPosition());
        telemetry.addData("s7", s7.getPosition());
        telemetry.addData("s8", s8.getPosition());
        telemetry.addData("s9", s9.getPosition());
        telemetry.addData("s10", s10.getPosition());
        telemetry.addData("s11", s11.getPosition());
        telemetry.addData("s12", s12.getPosition());
        telemetry.update();
    }
}
