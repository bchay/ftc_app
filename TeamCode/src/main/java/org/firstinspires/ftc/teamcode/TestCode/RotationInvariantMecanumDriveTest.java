package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

@TeleOp(name = "IMU Mecanum", group = "Test Code")
public class RotationInvariantMecanumDriveTest extends LinearOpMode {
    private DcMotor motorLeftFront;
    private DcMotor motorLeftBack;
    private DcMotor motorRightFront;
    private DcMotor motorRightBack;

    private BNO055IMU imu;

    private double desiredHeading = 0;
    private double strafeAngle = 0;

    public void runOpMode() {
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
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu 1");
        imu.initialize(parameters);

        telemetry.addData("Please press start.", "");
        telemetry.update();
        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        while(opModeIsActive()) {
            Orientation orientation = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            double robotHeading = (orientation.thirdAngle + 360) % 360;

            if(Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y) >= 1) {
                desiredHeading = (((Math.toDegrees(Math.atan2(gamepad1.right_stick_y, -gamepad1.right_stick_x)) + 360) % 360) + 90) % 360;
            }

            if(Math.abs(gamepad1.left_stick_x + gamepad1.left_stick_y) > 0) {
                strafeAngle = (((Math.toDegrees(Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x)) + 360) % 360)) - 45;
                strafeAngle += robotHeading;
                strafeAngle = strafeAngle % 360;

                double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);

                motorLeftFront.setPower(r * Math.cos(Math.toRadians(strafeAngle)));
                motorRightFront.setPower(r * Math.sin(Math.toRadians(strafeAngle)));
                motorLeftBack.setPower(r * Math.sin(Math.toRadians(strafeAngle)));
                motorRightBack.setPower(r * Math.cos(Math.toRadians(strafeAngle)));
            } else if(gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0) {
                if (Math.abs(robotHeading - desiredHeading) > 180) {
                    if (robotHeading < desiredHeading) {
                        //Turn right
                        motorLeftFront.setPower(Range.clip(Math.abs((robotHeading + desiredHeading - 360)) / 90, .4, 1));
                        motorLeftBack.setPower(Range.clip(Math.abs((robotHeading + desiredHeading - 360)) / 90, .4, 1));
                        motorRightFront.setPower(-Range.clip(Math.abs((robotHeading + desiredHeading - 360)) / 90, .4, 1));
                        motorRightBack.setPower(-Range.clip(Math.abs((robotHeading + desiredHeading - 360)) / 90, .4, 1));
                    } else {
                        //Turn left
                        motorLeftFront.setPower(-Range.clip(Math.abs((robotHeading + desiredHeading - 360)) / 90, .4, 1));
                        motorLeftBack.setPower(-Range.clip(Math.abs((robotHeading + desiredHeading - 360)) / 90, .4, 1));
                        motorRightFront.setPower(Range.clip(Math.abs((robotHeading + desiredHeading - 360)) / 90, .4, 1));
                        motorRightBack.setPower(Range.clip(Math.abs((robotHeading + desiredHeading - 360)) / 90, .4, 1));
                    }
                } else if (Math.abs(robotHeading - desiredHeading) > 3) { //Robot not within acceptable margin of error from desired heading
                    if (robotHeading < desiredHeading) {
                        //Turn left
                        motorLeftFront.setPower(-Range.clip(Math.abs(robotHeading - desiredHeading) / 90, .4, 1));
                        motorLeftBack.setPower(-Range.clip(Math.abs(robotHeading - desiredHeading) / 90, .4, 1));
                        motorRightFront.setPower(Range.clip(Math.abs(robotHeading - desiredHeading) / 90, .4, 1));
                        motorRightBack.setPower(Range.clip(Math.abs(robotHeading - desiredHeading) / 90, .4, 1));
                    } else {
                        //Turn Right
                        motorLeftFront.setPower(Range.clip(Math.abs(robotHeading - desiredHeading) / 90, .4, 1));
                        motorLeftBack.setPower(Range.clip(Math.abs(robotHeading - desiredHeading) / 90, .4, 1));
                        motorRightFront.setPower(-Range.clip(Math.abs(robotHeading - desiredHeading) / 90, .4, 1));
                        motorRightBack.setPower(-Range.clip(Math.abs(robotHeading - desiredHeading) / 90, .4, 1));
                    }
                } else {
                    //Currently at desired heading
                    motorLeftFront.setPower(0);
                    motorLeftBack.setPower(0);
                    motorRightFront.setPower(0);
                    motorRightBack.setPower(0);
                }
            }

            telemetry.addData("Strafe Angle", strafeAngle);
            telemetry.addData("Robot Heading", robotHeading);
            telemetry.addData("Desired heading", desiredHeading);
            telemetry.addData("Motor Power", Math.abs(motorLeftFront.getPower()));
            telemetry.update();

            /*
            //Assumes IMU initialization is forward from driver perspective
            if(Math.hypot(gamepad1.left_stick_y, gamepad1.left_stick_x) >= 1) {
                desiredHeading = (((Math.toDegrees(Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x)) + 360) % 360) + 90) % 360;
            }

            double robotHeading = (orientation.thirdAngle + 360) % 360;

            //Robot and desired headings are on opposite sides of the 0/360 discontinuity
            if (Math.abs(robotHeading - desiredHeading) > 180) {
                if (robotHeading < desiredHeading) {
                    //Turn right
                    motorLeftFront.setPower(Range.clip(Math.abs((robotHeading + desiredHeading - 360)) / 180, .3, 1));
                    motorLeftBack.setPower(Range.clip(Math.abs((robotHeading + desiredHeading - 360)) / 180, .3, 1));
                    motorRightFront.setPower(-Range.clip(Math.abs((robotHeading + desiredHeading - 360)) / 180, .3, 1));
                    motorRightBack.setPower(-Range.clip(Math.abs((robotHeading + desiredHeading - 360)) / 180, .3, 1));
                } else {
                    //Turn left
                    motorLeftFront.setPower(-Range.clip(Math.abs((robotHeading + desiredHeading - 360)) / 180, .3, 1));
                    motorLeftBack.setPower(-Range.clip(Math.abs((robotHeading + desiredHeading - 360)) / 180, .3, 1));
                    motorRightFront.setPower(Range.clip(Math.abs((robotHeading + desiredHeading - 360)) / 180, .3, 1));
                    motorRightBack.setPower(Range.clip(Math.abs((robotHeading + desiredHeading - 360)) / 180, .3, 1));
                }
            } else if (Math.abs(robotHeading - desiredHeading) > 3) { //Robot not within acceptable margin of error from desired heading
                if (robotHeading < desiredHeading) {
                    //Turn left
                    motorLeftFront.setPower(-Range.clip(Math.abs(robotHeading - desiredHeading) / 180, .3, 1));
                    motorLeftBack.setPower(-Range.clip(Math.abs(robotHeading - desiredHeading) / 180, .3, 1));
                    motorRightFront.setPower(Range.clip(Math.abs(robotHeading - desiredHeading) / 180, .3, 1));
                    motorRightBack.setPower(Range.clip(Math.abs(robotHeading - desiredHeading) / 180, .3, 1));
                } else {
                    //Turn Right
                    motorLeftFront.setPower(Range.clip(Math.abs(robotHeading - desiredHeading) / 180, .3, 1));
                    motorLeftBack.setPower(Range.clip(Math.abs(robotHeading - desiredHeading) / 180, .3, 1));
                    motorRightFront.setPower(-Range.clip(Math.abs(robotHeading - desiredHeading) / 180, .3, 1));
                    motorRightBack.setPower(-Range.clip(Math.abs(robotHeading - desiredHeading) / 180, .3, 1));
                }
            } else {
                //Currently at desired heading
                motorLeftFront.setPower(0);
                motorLeftBack.setPower(0);
                motorRightFront.setPower(0);
                motorRightBack.setPower(0);
            }

            telemetry.addData("Joystick Heading", desiredHeading);
            telemetry.addData("Robot Heading Final", robotHeading);
            telemetry.addData("Turn Direction", Math.abs(motorLeftFront.getPower() - 1) < .01 ? "Right" : "Left");
            telemetry.addData("Left Power", motorLeftFront.getPower());
            telemetry.addData("Hypotenuse", Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y));
            telemetry.update();
            */
        }
    }
}
