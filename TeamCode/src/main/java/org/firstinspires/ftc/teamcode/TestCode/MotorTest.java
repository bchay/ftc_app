package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

@TeleOp(name = "Drivetrain Encoder Test", group = "Test Code")
public class MotorTest extends OpMode {
    DcMotor motorLeftFront;
    DcMotor motorLeftBack;
    DcMotor motorRightFront;
    DcMotor motorRightBack;

    public void init() {
        motorLeftFront = hardwareMap.dcMotor.get("left front");
        motorLeftBack = hardwareMap.dcMotor.get("left back");
        motorRightFront = hardwareMap.dcMotor.get("right front");
        motorRightBack = hardwareMap.dcMotor.get("right back");

        motorLeftFront.setDirection(FORWARD);
        motorLeftBack.setDirection(FORWARD);
        motorRightFront.setDirection(REVERSE);
        motorRightBack.setDirection(REVERSE);

        motorLeftFront.setMode(STOP_AND_RESET_ENCODER);
        motorLeftBack.setMode(STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(STOP_AND_RESET_ENCODER);
        motorRightBack.setMode(STOP_AND_RESET_ENCODER);

        motorLeftFront.setMode(RUN_USING_ENCODER); //Autonomous methods that need a different mode will set it, RUN_WITHOUT_ENCODER used for teleop
        motorLeftBack.setMode(RUN_USING_ENCODER);
        motorRightFront.setMode(RUN_USING_ENCODER);
        motorRightBack.setMode(RUN_USING_ENCODER);
    }

    public void loop() {
        telemetry.addData("Left Front", motorLeftFront.getCurrentPosition());
        telemetry.addData(" Left Back", motorLeftBack.getCurrentPosition());
        telemetry.addData(" Right Front", motorRightFront.getCurrentPosition());
        telemetry.addData("Right Back", motorRightBack.getCurrentPosition());
        telemetry.update();
    }
}
