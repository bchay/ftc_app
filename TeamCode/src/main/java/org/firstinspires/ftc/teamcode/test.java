 package org.firstinspires.ftc.teamcode;
 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;//for automatic mode
 import com.qualcomm.robotcore.eventloop.opmode.TeleOp;//for using controller(Teleop)
 import com.qualcomm.robotcore.hardware.DcMotor;//to control motors
 import com.qualcomm.robotcore.hardware.HardwareMap;//not needed apparently but good to have
 import com.qualcomm.robotcore.hardware.Servo;//to control servos
 import com.qualcomm.robotcore.util.Range;

 /**
 * Created by RoboticsClub on 10/31/2016.
 */

@TeleOp(name="I like turtles")

public class test extends LinearOpMode {

    Servo test;
    double pos = 0.5;

    DcMotor motorRight1;
    DcMotor motorRight2;

    DcMotor motorLeft1;
    DcMotor motorLeft2;

    DcMotor motorBall;

    public void runOpMode() throws InterruptedException {
        motorRight1 = hardwareMap.dcMotor.get("right1"); //Value from hardware map of android app
        motorLeft1 = hardwareMap.dcMotor.get("left1"); //Value from hardware map of android app

        motorRight2 = hardwareMap.dcMotor.get("right2"); //Value from hardware map of android app
        motorLeft2 = hardwareMap.dcMotor.get("left2"); //Value from hardware map of android app

        test = hardwareMap.servo.get("servo1");
        test = hardwareMap.servo.get("servo2");

        motorBall = hardwareMap.dcMotor.get("Ball"); //make sure to name it ball on the app

        motorRight1.setDirection(DcMotor.Direction.REVERSE); //These motors go backwards normally, so set them to reverse
        motorRight2.setDirection(DcMotor.Direction.REVERSE); //These motors go backwards normally, so set them to reverse

        telemetry.addData("Revision", "4");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            motorRight1.setPower(0.8 * gamepad1.right_stick_y);
            motorRight2.setPower(0.8 * gamepad1.right_stick_y);

            motorLeft1.setPower(0.8 * gamepad1.left_stick_y);
            motorLeft2.setPower(0.8 * gamepad1.left_stick_y);

            if (gamepad1.x && pos < 1) { //if button x is pressed
                pos += 0.01;
            } else if (gamepad1.b && pos > 0) { //if button b is pressed
                pos -= 0.01;
            }

            telemetry.addData("Servo Position", "%5.2f", pos);
            telemetry.update();
            test.setPosition(pos);
            Thread.sleep(100);

            if (gamepad1.a) { //Set to full power immediately
                motorBall.setPower(1);
            }

            if (gamepad1.y && motorBall.getPower() < .8 ) { //Slowly increment speed
                motorBall.setPower(Range.clip(motorBall.getPower() + 0.0005, 0, .8));
            } else if(!gamepad1.y && motorBall.getPower() > 0) { //Decrement power
                motorBall.setPower(Range.clip(motorBall.getPower() - 0.0005, 0, 1));
            }

        }
    }
}