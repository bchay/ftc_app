package org.firstinspires.ftc.teamcode.TestCode;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * This program is an example of using the Android SharedPreferences API to save the robot heading after the autonomous period.
 * It is to be used with the rotation invariant mechanum drive code, which requires the initial heading at the beginning the teleop period.
 * This creates a Thread to write to the SharedPreferences file without slowing down the loop, and to ensure that the code is correctly saved even if the OpMode stops.
 * 
 */
@Autonomous(name = "Save Autonomous Heading", group = "Test Code")
public class AutoSaveHeading extends LinearOpMode {
    private BNO055IMU imu;

    public void runOpMode() {
        final SharedPreferences sharedPreferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);
        final SharedPreferences.Editor editor = sharedPreferences.edit();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu 1");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        while(!isStarted()) {
            telemetry.addData("Saved IMU Heading", sharedPreferences.getFloat("org.firstinspires.ftc.teamcode.AutonomousHeading", 10));
            telemetry.addData("IMU Heading", imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
            telemetry.addData("IMU Transformed", (imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle + 360) % 360);
            telemetry.addData("Joystick theta", Math.toDegrees(Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x)));
            telemetry.addData("Joystick transformed", (((Math.toDegrees(Math.atan2(gamepad1.right_stick_y, -gamepad1.right_stick_x)) + 360) % 360) + 90) % 360);
            telemetry.addData("Strafe Angle Initial", (Math.toDegrees(Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x))));
            telemetry.addData("Strafe Angle Transformed", ((((Math.toDegrees(Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x)) + 360) % 360)) - 45) % 360);
            telemetry.update();
        }

        ElapsedTime time = new ElapsedTime();
        while(opModeIsActive()) {
            telemetry.addData("Time", time.seconds());
            telemetry.addData("IMU Heading", imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
            telemetry.update();
        }

        new Thread(new Runnable() {
            @Override
            public void run() {
                if(Math.abs(imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle) - 2 > 0) {
                    editor.putFloat("org.firstinspires.ftc.teamcode.AutonomousHeading", imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
                    editor.commit();
                }
            }
        }).start();
    }
}
