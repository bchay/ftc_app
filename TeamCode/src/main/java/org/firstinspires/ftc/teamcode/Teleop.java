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
public class Teleop extends OpMode {
    //Hardware Declaration
    private DcMotor motorLeft; //Controls both left side motors
    private DcMotor motorRight; //Controls both right side motors

    private DcMotor ballShooter;
    private DcMotor ballLift;

    private DcMotor verticalSlide;

    private Servo buttonPresserLeft;
    private Servo buttonPresserRight;

    private Servo ballKicker;
    private Servo ballStopper;

    private Servo capBallLiftLeft;
    private Servo capBallLiftRight;

    //Variables
    private final double BUTTON_PRESSER_LEFT_IN = 0;
    private final double BUTTON_PRESSER_RIGHT_IN = .6;

    //Starting positions of ball manipulation servos
    private final double BALL_KICKER_DOWN = .65;
    private final double BALL_STOPPER_OUT = .5;

    private final double BALL_KICKER_UP = .3;
    private final double BALL_STOPPER_IN = .25;

    private final double LEFT_CAP_BALL_LIFT_UP = .15;
    private final double RIGHT_CAP_BALL_LIFT_UP = .85;

    //Variables for toggle
    boolean ballShooterOn = true;
    boolean ballShooterTriggered = false;

    private double motorPower = .23;

    public void init() {
        motorLeft = hardwareMap.dcMotor.get("left");
        motorRight = hardwareMap.dcMotor.get("right");

        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        ballShooter = hardwareMap.dcMotor.get("ball_shooter");
        ballShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ballShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        ballLift = hardwareMap.dcMotor.get("ball_lift");

        //verticalSlide = hardwareMap.dcMotor.get("vertical_slide");

        buttonPresserRight = hardwareMap.servo.get("button_right");
        buttonPresserLeft = hardwareMap.servo.get("button_left");

        ballKicker = hardwareMap.servo.get("ball_kicker");
        ballStopper = hardwareMap.servo.get("ball_stopper");

        capBallLiftLeft = hardwareMap.servo.get("cap_ball_left");
        capBallLiftRight = hardwareMap.servo.get("cap_ball_right");

        //Set initial servo positions
        buttonPresserLeft.setPosition(BUTTON_PRESSER_LEFT_IN);
        buttonPresserRight.setPosition(BUTTON_PRESSER_RIGHT_IN);

        ballKicker.setPosition(BALL_KICKER_DOWN);
        ballStopper.setPosition(BALL_STOPPER_OUT);

        capBallLiftLeft.setPosition(LEFT_CAP_BALL_LIFT_UP);
        capBallLiftRight.setPosition(RIGHT_CAP_BALL_LIFT_UP);
    }

    public void loop() {

        //Set motor power
        motorLeft.setPower(-gamepad1.left_stick_y);
        motorRight.setPower(-gamepad1.right_stick_y);

        //Start of gamepad 2 (Operator) controls
        ballLift.setPower(gamepad2.left_stick_y);

        //Set servo positions based on gamepad input
        if(gamepad2.y) {
            buttonPresserLeft.setPosition(Range.clip(buttonPresserLeft.getPosition() + .01, 0, 1));
        } else if(gamepad2.a) {
            buttonPresserLeft.setPosition(Range.clip(buttonPresserLeft.getPosition() - .01, 0, 1));
        }

        if(gamepad2.x) {
            buttonPresserRight.setPosition(Range.clip(buttonPresserRight.getPosition() + .01, 0, 1));
        } else if(gamepad2.b) {
            buttonPresserRight.setPosition(Range.clip(buttonPresserRight.getPosition() - .01, 0, 1));
        }

        if (gamepad2.left_bumper) ballKicker.setPosition(BALL_KICKER_UP);
        else ballKicker.setPosition(BALL_KICKER_DOWN);

        if(gamepad2.dpad_left) ballStopper.setPosition(BALL_STOPPER_IN);
        else ballStopper.setPosition(BALL_STOPPER_OUT);

        if(gamepad2.dpad_up) {
            capBallLiftLeft.setPosition(Range.clip(capBallLiftLeft.getPosition() + .01, 0, 1));
            capBallLiftRight.setPosition(Range.clip(capBallLiftRight.getPosition() - .01, 0, 1));
        }

        if(gamepad2.dpad_down) {
            capBallLiftLeft.setPosition(Range.clip(capBallLiftLeft.getPosition() + .01, 0, 1));
            capBallLiftRight.setPosition(Range.clip(capBallLiftRight.getPosition() - .01, 0, 1));
        }

        //TODO Lift cap ball


        //Toggle shooter
        if (gamepad2.dpad_right && !ballShooterTriggered) {
            ballShooterTriggered = true;
            if(ballShooterOn) ballShooter.setPower(getShooterPower());
            else ballShooter.setPower(0);
            ballShooterOn = !ballShooterOn;
        } else if(!gamepad2.dpad_right) ballShooterTriggered = false;

        if(gamepad1.dpad_up) motorPower = Range.clip(motorPower + .001, 0, .4);
        if(gamepad1.dpad_down) motorPower = Range.clip(motorPower - .001, 0, .4);

        telemetry.addData("Shooter motor power", ballShooter.getPower());
        telemetry.addData("Shooter power", motorPower);
        telemetry.addData("Voltage: ", this.hardwareMap.voltageSensor.iterator().next().getVoltage());
    }

    private double getShooterPower() {
        return -motorPower; //.7;

        /*
            12.78 = .28
            12.76 = .265
            13.00 = .226
         */
    }
}