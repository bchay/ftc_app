package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "Color Sensor Test", group = "Teleop")
public class ColorSensorTest extends OpMode {

    private ColorSensor colorSensor;
    private float[] hsv = {0F, 0F, 0F};

    public void init() {
        colorSensor = hardwareMap.colorSensor.get("color");
        colorSensor.enableLed(false);
    }

    public void loop() {
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsv);

        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());
        telemetry.addData("Alpha", colorSensor.alpha());

        telemetry.addData("Hue", hsv[0]);
        telemetry.addData("Value", hsv[1]);
        telemetry.addData("Saturation", hsv[2]);
    }
}