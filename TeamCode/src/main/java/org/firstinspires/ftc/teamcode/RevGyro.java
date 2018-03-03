package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class RevGyro {
    private BNO055IMU imu;

    private double integratedHeading = 0;
    private double headingOffset = 0;

    RevGyro(HardwareMap hardwareMap) {
        imu = hardwareMap.get(BNO055IMU.class, "imu 1");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu.initialize(parameters);
    }


    /*
    This method scales the input from the REV Hub BNO055 IMU, which is in the range [-179, 179], to the range [0, 359].
     */
    private double getScaledHeading() {
        double heading = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle;
        return (heading + 360) % 360;
    }
}
