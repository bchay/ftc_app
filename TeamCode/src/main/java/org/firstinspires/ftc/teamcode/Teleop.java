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

Driver Two - Operations - Gamepad 2
    X: Button Presser in
    B: Button Presser out
*/

@TeleOp(name = "Teleop")
public class Teleop extends OpModeBase { //Teleop is a LinearOpMode so it can extend the same base class as autonomous
    public void runOpMode() {
        super.runOpMode(); //Configure hardware
        waitForStart();

        motorLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        while (opModeIsActive()) {
            //Set motor power
            if(gamepad1.a) motorMax = .3; //Enables slow mode
            if(gamepad1.b) motorMax = 1; //Enables slow mode

            motorLeftFront.setPower(Range.clip(gamepad1.left_stick_y, -motorMax, motorMax));
            motorLeftBack.setPower(Range.clip(gamepad1.left_stick_y, -motorMax, motorMax));
            motorRightFront.setPower(Range.clip(gamepad1.right_stick_y, -motorMax, motorMax));
            motorRightBack.setPower(Range.clip(gamepad1.right_stick_y, -motorMax, motorMax));

            //Set servo positions based on gamepad input
            if (gamepad2.x) { //In
                buttonPresser.setPosition(Range.clip(buttonPresser.getPosition() - .01, BUTTON_PRESSER_IN, BUTTON_PRESSER_OUT));
            } else if (gamepad2.b) {
                buttonPresser.setPosition(Range.clip(buttonPresser.getPosition() + .01, BUTTON_PRESSER_IN, BUTTON_PRESSER_OUT));
            }

            telemetry.addData("Left Motor Power", motorLeftFront.getPower());
            telemetry.addData("Right Motor Power", motorRightFront.getPower());
            telemetry.addData("Button Presser", buttonPresser.getPosition());
            telemetry.addData("Voltage: ", this.hardwareMap.voltageSensor.iterator().next().getVoltage());
            telemetry.update();
        }
    }
}