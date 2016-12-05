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
    Right Joystick: Move right side motors
    Left Joystick: Move left side motors

Driver Two - Operations - Gamepad 2
    X: Button Presser Left
    B: Button Presser right

    Right Trigger: Vertical Slide up
    Left Trigger: Vertical Slide down
 */

@TeleOp(name = "Teleop")
public class Teleop extends OpModeBase { //Teleop is a LinearOpMode so that both it and AutonomousCode can extend the same superclass
    public void runOpMode() {
        super.runOpMode();
        waitForStart();

        motorLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        while (opModeIsActive()) {
            //Set motor power
            if(gamepad1.a) motorMax = .3; //Enables slow mode
            if(gamepad1.b) motorMax = 1; //Enables slow mode

            motorLeftFront.setPower(Range.clip(gamepad1.left_stick_y, -motorMax, motorMax)); //Motors are reversed in OpModeBase for autonomous
            motorLeftBack.setPower(Range.clip(gamepad1.left_stick_y, -motorMax, motorMax));
            motorRightFront.setPower(Range.clip(gamepad1.right_stick_y, -motorMax, motorMax));
            motorRightBack.setPower(Range.clip(gamepad1.right_stick_y, -motorMax, motorMax));

            //Set servo positions based on gamepad input
            if (gamepad2.x) { //Left
                buttonPresser.setPosition(Range.clip(buttonPresser.getPosition() - .01, BUTTON_PRESSER_LEFT, BUTTON_PRESSER_RIGHT));
            } else if (gamepad2.b) {
                buttonPresser.setPosition(Range.clip(buttonPresser.getPosition() + .01, BUTTON_PRESSER_LEFT, BUTTON_PRESSER_RIGHT));
            }

            if(gamepad2.right_trigger > .5) {
                verticalSlideOne.setPower(1);
                verticalSlideTwo.setPower(1);
            } else if(gamepad2.left_trigger > .5) {
                verticalSlideOne.setPower(-1);
                verticalSlideTwo.setPower(-1);
            } else {
                verticalSlideOne.setPower(0);
                verticalSlideTwo.setPower(0);
            }

            telemetry.addData("Left Motor Power", motorLeftFront.getPower());
            telemetry.addData("Right Motor Power", motorRightFront.getPower());
            telemetry.addData("Button Presser", buttonPresser.getPosition());
            telemetry.addData("Voltage: ", this.hardwareMap.voltageSensor.iterator().next().getVoltage());
            telemetry.update();
        }
    }
}