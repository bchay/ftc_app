package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

/*
GAMEPAD MAPPINGS:

Driver One - Movement - Gamepad 1
    Right Joystick: Move right side motors
    Left Joystick: Move left side motors

Driver Two - Operations - Gamepad 2
    A: Left Button Presser Down
    B: Right Button Presser Down
    X: Right Button Presser Up
    Y: Left Buton Presser Up

    Left Joystick: Ball Lift

    Left Bumper Pressed: Ball Kicker Down
    Left Bumper Not Pressed: Ball Kicker Up

    Dpad Left Pressed: Ball Stopper In
    Dpad Left Not Pressed: Ball Stopper Out

    Right Bumper: Toggle Lift and Shooter
    Dpad Right: Toggle Ball Shooter
    Dpad Up: Cap ball lift up
    Dpad Down: Cap ball lift down
 */

@TeleOp(name = "Teleop")
public class Teleop extends OpModeBase {

    public void runOpMode() {
        while (opModeIsActive()) {
            //Set motor power
            motorLeft.setPower(-gamepad1.left_stick_y);
            motorRight.setPower(-gamepad1.right_stick_y);

            //Start of gamepad 2 (Operator) controls
            ballLift.setPower(gamepad2.left_stick_y);

            //Set servo positions based on gamepad input
            if (gamepad2.y) {
                buttonPresserLeft.setPosition(Range.clip(buttonPresserLeft.getPosition() + .01, 0, 1));
            } else if (gamepad2.a) {
                buttonPresserLeft.setPosition(Range.clip(buttonPresserLeft.getPosition() - .01, 0, 1));
            }

            if (gamepad2.x) {
                buttonPresserRight.setPosition(Range.clip(buttonPresserRight.getPosition() + .01, 0, 1));
            } else if (gamepad2.b) {
                buttonPresserRight.setPosition(Range.clip(buttonPresserRight.getPosition() - .01, 0, 1));
            }

            if (gamepad2.left_bumper) ballKicker.setPosition(BALL_KICKER_UP);
            else ballKicker.setPosition(BALL_KICKER_DOWN);

            if (gamepad2.dpad_left) ballStopper.setPosition(BALL_STOPPER_IN);
            else ballStopper.setPosition(BALL_STOPPER_OUT);

            //Toggle shooter
            if (gamepad2.dpad_right && !ballShooterTriggered) {
                ballShooterTriggered = true;
                if (ballShooterOn) ballShooter.setPower(getShooterPower());
                else ballShooter.setPower(0);
                ballShooterOn = !ballShooterOn;
            } else if (!gamepad2.dpad_right) ballShooterTriggered = false;

            if (gamepad1.dpad_up) shooterPower = Range.clip(shooterPower + .001, 0, .4);
            if (gamepad1.dpad_down) shooterPower = Range.clip(shooterPower - .001, 0, .4);

            telemetry.addData("Shooter motor power", ballShooter.getPower());
            telemetry.addData("Shooter power", shooterPower);
            telemetry.addData("Voltage: ", this.hardwareMap.voltageSensor.iterator().next().getVoltage());
        }
    }

    private double getShooterPower() { //Will eventually return power based on battery voltage
        return -shooterPower;
    }
}