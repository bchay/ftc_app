package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Servo Test", group = "Test Code")
public class ServoTest extends OpMode {
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
        s1 = hardwareMap.servo.get("1");
        s2 = hardwareMap.servo.get("2");
        s3 = hardwareMap.servo.get("3");
        s4 = hardwareMap.servo.get("4");
        s5 = hardwareMap.servo.get("5");
        s6 = hardwareMap.servo.get("6");
        s7 = hardwareMap.servo.get("7");
        s8 = hardwareMap.servo.get("8");
        s9 = hardwareMap.servo.get("9");
        s10 = hardwareMap.servo.get("10");
        s11 = hardwareMap.servo.get("11");
        s12 = hardwareMap.servo.get("12");
    }

    public void loop() {
        s1.setPosition(.5);
        s2.setPosition(.5);
        s3.setPosition(.5);
        s4.setPosition(.5);
        s5.setPosition(.5);
        s6.setPosition(.5);
        s7.setPosition(.5);
        s8.setPosition(.5);
        s9.setPosition(.5);
        s10.setPosition(.5);
        s11.setPosition(.5);
        s12.setPosition(.5);

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
