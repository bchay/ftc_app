package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

@TeleOp(name = "ODS Test", group = "Test Code")
public class OpticalDistanceSensorTest extends OpMode {
    public OpticalDistanceSensor ods;

    public void init() {
        ods = hardwareMap.opticalDistanceSensor.get("ods");
    }

    public void loop() {
        telemetry.addData("Raw", ods.getRawLightDetected());
        telemetry.addData("Light", ods.getLightDetected());
    }
}