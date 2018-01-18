package org.firstinspires.ftc.teamcode.TestCode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "Color Sensor Test", group = "Test Code")
public class ColorSensorTest extends LinearOpMode {
    private ColorSensor colorSensor;

    public void runOpMode() {
        colorSensor = hardwareMap.colorSensor.get("color");
        colorSensor.enableLed(true);

        waitForStart();

        while(opModeIsActive()) {
            float[] hsv = {0F, 0F, 0F};

            Color.RGBToHSV(colorSensor.red() * 255, colorSensor.green() * 255, colorSensor.blue() * 255, hsv);

            if(hsv[2] < 10) telemetry.addData("Color", "Unknown");
            else if ((hsv[0] < 30 || hsv[0] > 340) && hsv[1] > .2) telemetry.addData("Color", "Red");
            else if ((hsv[0] > 170 && hsv[0] < 260) && hsv[1] > .2) telemetry.addData("Color", "Blue");
            else telemetry.addData("Color", "Unknown");

            telemetry.addData("Hue", hsv[0]);
            telemetry.addData("Saturation", hsv[1]);
            telemetry.addData("Value", hsv[2]);

            telemetry.addData("Red", colorSensor.red());
            telemetry.addData("Green", colorSensor.blue());
            telemetry.addData("Blue", colorSensor.green());

            telemetry.update();
        }
    }
}
