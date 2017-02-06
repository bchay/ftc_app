package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
GAMEPAD MAPPINGS:

Driver One - Movement - Gamepad 1
    Tank Drive:
    Right Joystick: Right drivetrain motors
    Left Joystick: Left drivetrain motors

    A: Enable slow mode
    B: Disable slow mode

    Right Trigger: Intake
    Left Trigger: Shooter

    X: Button presser out
    B: Button presser in
    A: Ball stop up
    Y: Ball stop down

Driver Two - Operations - Gamepad 2

*/

@TeleOp(name = "Teleop")
public class Teleop extends OpModeBase { //Teleop is a LinearOpMode so it can extend the same base class as autonomous
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

            if(gamepad1.b) { //Enables slow mode
                motorMax = .3;
            } else if(gamepad1.a) {
                motorMax = 1;
            }

            if(motorMax == .3) telemetry.addData("Slow mode", "true");
            else telemetry.addData("Fast mode", "true");

            motorLeftFront.setPower(Range.clip(gamepad1.left_stick_y, -motorMax, motorMax));
            motorLeftBack.setPower(Range.clip(gamepad1.left_stick_y, -motorMax, motorMax));
            motorRightFront.setPower(Range.clip(gamepad1.right_stick_y, -motorMax, motorMax));
            motorRightBack.setPower(Range.clip(gamepad1.right_stick_y, -motorMax, motorMax));

            //Set servo positions based on gamepad input
            if (gamepad2.b) buttonPresser.setPosition(Range.clip(buttonPresser.getPosition() - .005, BUTTON_PRESSER_OUT, BUTTON_PRESSER_IN));
            else if (gamepad2.x) buttonPresser.setPosition(Range.clip(buttonPresser.getPosition() + .005, BUTTON_PRESSER_OUT, BUTTON_PRESSER_IN));

            shooter.setPower(gamepad2.left_trigger);
            intake.setPower(gamepad2.right_trigger);
            if (gamepad2.right_bumper) intake.setPower(-1);

            if(touchSensorBall.isPressed()) ballStop.setPosition(BALL_STOP_BLOCKED);
            else ballStop.setPosition(BALL_STOP_UP);

            telemetry.addData("Left Motor Power", motorLeftFront.getPower());
            telemetry.addData("Right Motor Power", motorRightFront.getPower());
            telemetry.addData("Button Presser", buttonPresser.getPosition());
            telemetry.addData("Ball stop", ballStop.getPosition());
            telemetry.addData("Voltage: ", this.hardwareMap.voltageSensor.iterator().next().getVoltage());
            telemetry.update();
        }
    }
}