package org.firstinspires.ftc.teamcode.TestCode;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.HashMap;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

/**
<p>
 * This program is a standalone opmode that is used as a demonstration of the rotation invariant mecanum drive system.
 * It uses the REV Expansion Hub IMU to determine the heading of the joystick, and will automatically apply transformations
 * to the joysticks to ensure that the robot moves from the driver's perspective. Additionally, the robot will automatically
 * correct itself so that it maintains the target heading.
 </p>

 <p>
  * The left joystick is used to strafe the robot in the XY plane from the perspective of the driver. This means that, regardless
 * of the robot's orientation, pushing the left joystick forward will cause the robot to move forward from the driver's perspective.
 * The left joystick changes the robot's target heading, and the robot will automatically take the shortest path to turn to
 * the new target.
 </p>


 <p>
 * If the robot is jostled in the middle of a match, it will autocorrect to the target heading. The X button can be used to reset
 * the zero position, which will help to counteract any gyroscope drift.
 </p>

 <p>
 * Finally, there is an optional autonomous correction factor that will allow the robot to have its initial target zero positon
 * be set to forward from the perspective of the driver, rather than defining the forward position to be from the zero
 * orientation of the robot's starting position at the end of autonomous.
 </p>
 */
@TeleOp(name = "IMU Mecanum", group = "Test Code")
public class RotationInvariantMecanumDriveTest extends LinearOpMode {
    private DcMotor motorLeftFront;
    private DcMotor motorLeftBack;
    private DcMotor motorRightFront;
    private DcMotor motorRightBack;

    private BNO055IMU imu;

    private double desiredHeading = 0;
    private double strafeAngle = 0;
    private double autonomousOffset;
    private double resetOffset = 0;
    private double correction = 0;

    private HashMap<String, Boolean> previousLoopValues = new HashMap<>();

    public void runOpMode() {
        final SharedPreferences sharedPreferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);

        autonomousOffset = (-90 + sharedPreferences.getFloat("org.firstinspires.ftc.teamcode.AutonomousHeading", 0) + 360) % 360; //Use base gyro value
        desiredHeading = autonomousOffset;

        motorLeftFront = hardwareMap.dcMotor.get("left front");
        motorLeftBack = hardwareMap.dcMotor.get("left back");
        motorRightFront = hardwareMap.dcMotor.get("right front");
        motorRightBack = hardwareMap.dcMotor.get("right back");

        motorLeftFront.setDirection(REVERSE);
        motorLeftBack.setDirection(REVERSE);
        motorRightFront.setDirection(FORWARD);
        motorRightBack.setDirection(FORWARD);

        motorLeftFront.setMode(STOP_AND_RESET_ENCODER);
        motorLeftBack.setMode(STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(STOP_AND_RESET_ENCODER);
        motorRightBack.setMode(STOP_AND_RESET_ENCODER);

        motorLeftFront.setMode(RUN_USING_ENCODER); //Autonomous methods that need a different mode will set it, RUN_WITHOUT_ENCODER used for teleop
        motorLeftBack.setMode(RUN_USING_ENCODER);
        motorRightFront.setMode(RUN_USING_ENCODER);
        motorRightBack.setMode(RUN_USING_ENCODER);

        motorLeftFront.setZeroPowerBehavior(BRAKE); //Default is FLOAT
        motorLeftBack.setZeroPowerBehavior(BRAKE);
        motorRightFront.setZeroPowerBehavior(BRAKE);
        motorRightBack.setZeroPowerBehavior(BRAKE);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu 1");
        imu.initialize(parameters);

        previousLoopValues.put("gamepad1.x", false);

        telemetry.addData("Please press start.", "");
        telemetry.update();
        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        while(opModeIsActive()) {
            Orientation orientation = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

            if(gamepad1.x && !previousLoopValues.get("gamepad1.x")) {
                resetOffset = (orientation.thirdAngle + 360) % 360; //Transform to interval [0, 360)
                autonomousOffset = 0;
                desiredHeading = 0;
            }

            double robotHeading = (orientation.thirdAngle + autonomousOffset + 360) % 360 - resetOffset;
            if(robotHeading < 0) robotHeading += 360;

            if(Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y) >= 1) {
                desiredHeading = (((Math.toDegrees(Math.atan2(gamepad1.right_stick_y, -gamepad1.right_stick_x)) + 360) % 360) + 90) % 360;
            }

            if(Math.abs(gamepad1.left_stick_x + gamepad1.left_stick_y) > 0) {
                strafeAngle = (((Math.toDegrees(Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x)) + 360) % 360)) - 45;
                strafeAngle += robotHeading;
                strafeAngle = strafeAngle % 360;

                double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);


                if (Math.abs(robotHeading - desiredHeading) > 180) {
                    if(robotHeading < desiredHeading) {
                        //Turn right
                        correction = Range.clip(Math.abs((robotHeading + desiredHeading - 360)) / 30, 0, 1);

                        motorLeftFront.setPower(Range.clip(r * Math.cos(Math.toRadians(strafeAngle)), -.9, .9) + correction);
                        motorLeftBack.setPower(Range.clip(r * Math.sin(Math.toRadians(strafeAngle)), -.9, .9) + correction);
                        motorRightFront.setPower(Range.clip(r * Math.sin(Math.toRadians(strafeAngle)), -.9, .9) - correction);
                        motorRightBack.setPower(Range.clip(r * Math.cos(Math.toRadians(strafeAngle)), -.9, .9) - correction);
                    } else {
                        //Turn left
                        correction = Range.clip(Math.abs((robotHeading + desiredHeading - 360)) / 30, 0, 1);

                        motorLeftFront.setPower(Range.clip(r * Math.cos(Math.toRadians(strafeAngle)), -.9, .9) - correction);
                        motorLeftBack.setPower(Range.clip(r * Math.sin(Math.toRadians(strafeAngle)), -.9, .9) - correction);
                        motorRightFront.setPower(Range.clip(r * Math.sin(Math.toRadians(strafeAngle)), -.9, .9) + correction);
                        motorRightBack.setPower(Range.clip(r * Math.cos(Math.toRadians(strafeAngle)), -.9, .9) + correction);
                    }
                } else if (Math.abs(robotHeading - desiredHeading) > 2) { //Robot not within acceptable margin of error from desired heading
                    if (robotHeading < desiredHeading) {
                        //Turn left
                        correction = Range.clip(Math.abs(robotHeading - desiredHeading) / 30, 0, 1);

                        motorLeftFront.setPower(Range.clip(r * Math.cos(Math.toRadians(strafeAngle)), -.9, .9) - correction);
                        motorLeftBack.setPower(Range.clip(r * Math.sin(Math.toRadians(strafeAngle)), -.9, .9) - correction);
                        motorRightFront.setPower(Range.clip(r * Math.sin(Math.toRadians(strafeAngle)), -.9, .9) + correction);
                        motorRightBack.setPower(Range.clip(r * Math.cos(Math.toRadians(strafeAngle)), -.9, .9) + correction);
                    } else {
                        //Turn Right
                        correction = Range.clip(Math.abs(robotHeading - desiredHeading) / 30, 0, 1);

                        motorLeftFront.setPower(Range.clip(r * Math.cos(Math.toRadians(strafeAngle)), -.9, .9) + correction);
                        motorLeftBack.setPower(Range.clip(r * Math.sin(Math.toRadians(strafeAngle)), -.9, .9) + correction);
                        motorRightFront.setPower(Range.clip(r * Math.sin(Math.toRadians(strafeAngle)), -.9, .9) - correction);
                        motorRightBack.setPower(Range.clip(r * Math.cos(Math.toRadians(strafeAngle)), -.9, .9) - correction);
                    }
                } else {
                    motorLeftFront.setPower(r * Math.cos(Math.toRadians(strafeAngle)));
                    motorLeftBack.setPower(r * Math.sin(Math.toRadians(strafeAngle)));
                    motorRightFront.setPower(r * Math.sin(Math.toRadians(strafeAngle)));
                    motorRightBack.setPower(r * Math.cos(Math.toRadians(strafeAngle)));
                }
            } else if(gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0) {
                if (Math.abs(robotHeading - desiredHeading) > 180) {
                    if(robotHeading < desiredHeading) {
                        //Turn right
                        motorLeftFront.setPower(Range.clip(Math.abs((robotHeading + desiredHeading - 360)) / 90, .3, 1));
                        motorLeftBack.setPower(Range.clip(Math.abs((robotHeading + desiredHeading - 360)) / 90, .3, 1));
                        motorRightFront.setPower(-Range.clip(Math.abs((robotHeading + desiredHeading - 360)) / 90, .3, 1));
                        motorRightBack.setPower(-Range.clip(Math.abs((robotHeading + desiredHeading - 360)) / 90, .3, 1));
                    } else {
                        //Turn left
                        motorLeftFront.setPower(-Range.clip(Math.abs((robotHeading + desiredHeading - 360)) / 90, .3, 1));
                        motorLeftBack.setPower(-Range.clip(Math.abs((robotHeading + desiredHeading - 360)) / 90, .3, 1));
                        motorRightFront.setPower(Range.clip(Math.abs((robotHeading + desiredHeading - 360)) / 90, .3, 1));
                        motorRightBack.setPower(Range.clip(Math.abs((robotHeading + desiredHeading - 360)) / 90, .3, 1));
                    }
                } else if (Math.abs(robotHeading - desiredHeading) > 2) { //Robot not within acceptable margin of error from desired heading
                    if (robotHeading < desiredHeading) {
                        //Turn left
                        motorLeftFront.setPower(-Range.clip(Math.abs(robotHeading - desiredHeading) / 90, .3, 1));
                        motorLeftBack.setPower(-Range.clip(Math.abs(robotHeading - desiredHeading) / 90, .3, 1));
                        motorRightFront.setPower(Range.clip(Math.abs(robotHeading - desiredHeading) / 90, .3, 1));
                        motorRightBack.setPower(Range.clip(Math.abs(robotHeading - desiredHeading) / 90, .3, 1));
                    } else {
                        //Turn Right
                        motorLeftFront.setPower(Range.clip(Math.abs(robotHeading - desiredHeading) / 90, .3, 1));
                        motorLeftBack.setPower(Range.clip(Math.abs(robotHeading - desiredHeading) / 90, .3, 1));
                        motorRightFront.setPower(-Range.clip(Math.abs(robotHeading - desiredHeading) / 90, .3, 1));
                        motorRightBack.setPower(-Range.clip(Math.abs(robotHeading - desiredHeading) / 90, .3, 1));
                    }
                } else {
                    //Currently at desired heading
                    motorLeftFront.setPower(0);
                    motorLeftBack.setPower(0);
                    motorRightFront.setPower(0);
                    motorRightBack.setPower(0);
                }
            }

            previousLoopValues.put("gamepad1.x", gamepad1.x);

            telemetry.addData("Autonomous Offset", autonomousOffset);
            telemetry.addData("Reset Offset", resetOffset);
            telemetry.addData("Strafe Angle", strafeAngle);
            telemetry.addData("Robot Heading", robotHeading);
            telemetry.addData("Original IMU heading", orientation.thirdAngle);
            telemetry.addData("Desired heading", desiredHeading);
            telemetry.addData("Motor Power", Math.abs(motorLeftFront.getPower()));
            telemetry.addData("Heading Error", Math.abs(robotHeading - desiredHeading));
            telemetry.addData("Correction", correction);
            telemetry.update();
        }
    }
}
