package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

@TeleOp(name = "Color Sensor Test", group = "Test Code")
public class ColorSensorTest extends OpMode {

    private ColorSensor colorSensor;
    private float[] hsv = {0F, 0F, 0F};

    public void init() {
        colorSensor = hardwareMap.colorSensor.get("color_front");
        I2cAddr i2CAddress = I2cAddr.create8bit(0x3c);
        colorSensor.setI2cAddress(i2CAddress);
        colorSensor.enableLed(false);
    }

    public void loop() {
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsv);

        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());
        telemetry.addData("Alpha", colorSensor.alpha());

        telemetry.addData("Hue", hsv[0]);
        telemetry.addData("Saturation", hsv[1]);
        telemetry.addData("Value", hsv[2]);

        if ((hsv[0] < 30 || hsv[0] > 340) && hsv[1] > .2) telemetry.addData("Color is RED", "");
        else if ((hsv[0] > 170 && hsv[0] < 260) && hsv[1] > .2) telemetry.addData("Color is BLUE", "");
        else telemetry.addData("Color cannot be determined.", "");
    }
}