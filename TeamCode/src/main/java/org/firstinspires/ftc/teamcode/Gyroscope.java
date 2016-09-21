package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Gyroscope Test", group = "Teleop")
public class Gyroscope extends OpMode {
    private ModernRoboticsI2cGyro gyro;

    public void init() {
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();
    }

    public void loop() {
        telemetry.addData("Heading: ", gyro.getHeading());
        telemetry.addData("Integrated Z Value: ", gyro.getIntegratedZValue()); //Provides +/- values for heading
    }
}