package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Gyroscope Test", group = "Test Code")
public class GyroscopeTest extends OpMode {
    private ModernRoboticsI2cGyro gyro;

    public void init() {
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();

        telemetry.addData("Gyroscope is calibrated.", "");
        telemetry.update();
    }

    public void loop() {
        System.out.println(gyro.getIntegratedZValue());
        telemetry.addData("Integrated Z Value: ", gyro.getIntegratedZValue()); //Provides +/- values for heading
    }
}