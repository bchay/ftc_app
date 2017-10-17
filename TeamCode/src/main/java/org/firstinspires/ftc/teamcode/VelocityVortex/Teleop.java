package org.firstinspires.ftc.teamcode.VelocityVortex;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

/*
GAMEPAD MAPPINGS:

Driver One - Movement - Gamepad 1
    Tank Drive:
    Right Joystick: Right drivetrain motors
    Left Joystick: Left drivetrain motors

    B: Toggle slow mode - 50% speed, 100% speed
    X: Toggle drivetrain direction - forward, reverse

Driver Two - Operations - Gamepad 2
        Right Trigger: Intake
        Left Trigger: Shooter
        Right Bumper: Intake

        X: Button presser out
        B: Button presser in
        A: Ball stop blocked
        Y: Ball stop up

        Left Bumper: Shoot one rotation

*/

/**
 * This is the MecanumTeleop code for the robot.
 */
@TeleOp(name = "Velocity Vortex MecanumTeleop")
public class Teleop extends OpModeBase { //MecanumTeleop is a LinearOpMode so it can extend the same base class as autonomous
    boolean previousLeftBumperState = false;

    boolean previousSpeedToggleState = false;

    boolean previousReverseDrivetrainState = false;
    boolean reverseDrivetrain = false;

    public void runOpMode() {
        super.runOpMode(); //Configure hardware

        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //Autonomous methods that need RUN_TO_POSITION will set the motors, RUN_USING_ENCODER is required for teleop and gyro turn
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            //Set motor power

            if(!previousSpeedToggleState && gamepad1.b) { //Just became pressed
                motorMax = motorMax == .5 ? 1 : .5; //Toggle motor max
                previousSpeedToggleState = true;
            } else if(!gamepad1.b && previousSpeedToggleState) {
                previousSpeedToggleState = false;
            }

            if(!previousReverseDrivetrainState && gamepad1.x) { //Just became pressed
                reverseDrivetrain = !reverseDrivetrain;
                previousReverseDrivetrainState = true;
            } else if(previousReverseDrivetrainState && !gamepad1.x) {
                previousReverseDrivetrainState = false;
            }

            if(!reverseDrivetrain) { //Normal mode
                motorLeftFront.setPower(Range.clip(gamepad1.left_stick_y, -motorMax, motorMax));
                motorLeftBack.setPower(Range.clip(gamepad1.left_stick_y, -motorMax, motorMax));
                motorRightFront.setPower(touchSensorBottom.isPressed() ? 0 : Range.clip(gamepad1.right_stick_y, -motorMax, motorMax));
                motorRightBack.setPower(touchSensorBottom.isPressed() ? 0 : Range.clip(gamepad1.right_stick_y, -motorMax, motorMax));
            } else {
                motorLeftFront.setPower(Range.clip(-gamepad1.right_stick_y, -motorMax, motorMax));
                motorLeftBack.setPower(Range.clip(-gamepad1.right_stick_y, -motorMax, motorMax));
                motorRightFront.setPower(Range.clip(-gamepad1.left_stick_y, -motorMax, motorMax));
                motorRightBack.setPower(Range.clip(-gamepad1.left_stick_y, -motorMax, motorMax));
            }




            //Second gamepad

            //Set servo positions based on gamepad input
            if (gamepad2.x) buttonPresser.setPosition(BUTTON_PRESSER_OUT);
            else if (gamepad2.b && !gamepad2.start) buttonPresser.setPosition(BUTTON_PRESSER_IN);

            shooter.setPower(gamepad2.left_trigger);
            intake.setPower(gamepad2.right_trigger);

            if(odsBallDetected()) { //Sets blocker to down, a overrides
                if(gamepad2.y) ballStop.setPosition(BALL_STOP_UP);
                else ballStop.setPosition(BALL_STOP_BLOCKED);
            } else { //No ball is loaded, blocker is set to up
                if(gamepad2.a) {
                    ballStop.setPosition(BALL_STOP_BLOCKED);
                } else {
                    ballStop.setPosition(BALL_STOP_UP);
                }
            }

            telemetry.addData("Left Motor Power", motorLeftFront.getPower());
            telemetry.addData("Right Motor Power", motorRightFront.getPower());
            telemetry.addData("Button Presser Position", buttonPresser.getPosition());
            telemetry.addData("Ball stop Position", ballStop.getPosition());
            telemetry.addData("Slow mode", motorMax != 1);
            telemetry.addData("Drivetrain forwards", !reverseDrivetrain);
            telemetry.addData("Shoooter position", shooter.getCurrentPosition());
            telemetry.addData("Voltage: ", this.hardwareMap.voltageSensor.iterator().next().getVoltage());
            telemetry.update();
        }
    }
}