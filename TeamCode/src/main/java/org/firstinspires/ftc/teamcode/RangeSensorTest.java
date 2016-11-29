package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Range Sensor Test", group = "Test Code")
public class RangeSensorTest extends OpMode {
    ModernRoboticsI2cRangeSensor range;

    public void init() {
        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");
    }

    public void loop() {
        telemetry.addData("Raw Optical", range.rawOptical());
        telemetry.addData("Optical CM", range.cmOptical());
        telemetry.addData("Raw Ultrasonic", range.rawUltrasonic());
        telemetry.addData("Ultrasonic CM", range.cmUltrasonic());
        telemetry.addData("Distance", range.getDistance(DistanceUnit.INCH));
    }
}
