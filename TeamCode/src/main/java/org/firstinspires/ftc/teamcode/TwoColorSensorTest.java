package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;


public class TwoColorSensorTest extends OpMode {
    ColorSensor colorSensorFront;
    ColorSensor colorSensorBottom;

    //Use MR Core Device Discovery to change address
    I2cAddr i2CAddressColorFront = I2cAddr.create8bit(0x3c);
    I2cAddr i2CAddressColorBottom = I2cAddr.create8bit(0x4c);

    public void init() {
        colorSensorFront = hardwareMap.colorSensor.get("color_sensor_front");
        colorSensorBottom = hardwareMap.colorSensor.get("color_sensor_bottom");

        //ColorFront reads beacon light and is in passive mode
        colorSensorFront.setI2cAddress(i2CAddressColorFront);
        colorSensorFront.enableLed(false);

        //ColorBottom reads white line and is in active mode
        colorSensorBottom.setI2cAddress(i2CAddressColorBottom);
        colorSensorBottom.enableLed(true);
    }

    public void loop() {
        telemetry.addData("Front color", colorSensorFront.argb());
        telemetry.addData("Bottom color", colorSensorBottom.argb());

    }
}