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
    Right Joystick: Move right side motors
    Left Joystick: Move left side motors

Driver Two - Operations - Gamepad 2
    X: Button Presser Left
    B: Button Presser right
 */

@TeleOp(name = "Teleop")
public class Teleop extends OpModeBase { //Teleop is a LinearOpMode so that both it and AutonomousCode can extend the same superclass
    public void runOpMode() {
        super.runOpMode();
        while (opModeIsActive()) {
            //Set motor power
            motorLeftFront.setPower(gamepad1.left_stick_y);
            motorLeftBack.setPower(gamepad1.left_stick_y);
            motorRightFront.setPower(gamepad1.right_stick_y);
            motorRightBack.setPower(gamepad1.right_stick_y);

            //Set servo positions based on gamepad input
            if (gamepad2.x) { //Left
                buttonPresser.setPosition(Range.clip(buttonPresser.getPosition() - .01, BUTTON_PRESSER_LEFT, BUTTON_PRESSER_RIGHT));
            } else if (gamepad2.b) {
                buttonPresser.setPosition(Range.clip(buttonPresser.getPosition() + .01, BUTTON_PRESSER_LEFT, BUTTON_PRESSER_RIGHT));
            }

            telemetry.addData("Left Motor Power", motorLeftFront.getPower());
            telemetry.addData("Right Motor Power", motorRightFront.getPower());
            telemetry.addData("Button Presser", buttonPresser.getPosition());
            telemetry.addData("Voltage: ", this.hardwareMap.voltageSensor.iterator().next().getVoltage());
            telemetry.addData("cm", "%.2f cm", range.getDistance(DistanceUnit.CM));

            telemetry.update();
        }
    }
}