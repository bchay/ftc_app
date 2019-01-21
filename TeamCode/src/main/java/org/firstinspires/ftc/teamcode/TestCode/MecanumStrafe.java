package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Allows the robot to strafe at a specified angle for a specified distance.
 */
@Autonomous(name = "Strafe at Angle")
public class MecanumStrafe extends LinearOpMode {
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    private int ENCODER_THRESHOLD = 5;

    public void runOpMode() {
        leftFront = hardwareMap.dcMotor.get("left front");
        leftBack = hardwareMap.dcMotor.get("right back");
        rightFront = hardwareMap.dcMotor.get("right front");
        rightBack = hardwareMap.dcMotor.get("right back");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        strafe(24, 45);
    }

    /**
     * Causes the robot to strafe in the indicated direction
     *
     * @param distance the distance in ticks that the robot should strafe
     * @param angle the angle, in degrees, at which the robot should strafe
     *              measured counterclockwise with 0/360 degrees being the positive x axis
     */
    public void strafe(int distance, double angle) {
        angle *= Math.PI / 180; //Convert to radians
        angle -= Math.PI / 4; //Transform so that 0 represents positive x axis

        //Power depends on angle - strafe(45 deg) means LFP = 1; strafe(90 deg) means LFP = .707
        double leftFrontPower = Math.cos(angle);
        double leftBackPower = Math.sin(angle);
        double rightFrontPower = Math.sin(angle);
        double rightBackPower = Math.cos(angle);

        //Normalize power so that maximum is 1, rather than being dependent on the angle
        //strafe(45 deg) means LFP = 1; strafe(90 deg) means LFP = 1
        //This division still results in the correct angle after vectors are added
        double maxPower = Math.max(leftFrontPower, leftBackPower); //right powers are same as left (sine/cosine)
        leftFrontPower /= maxPower;
        leftBackPower /= maxPower;
        rightFrontPower /= maxPower;
        rightBackPower /= maxPower;

        //RUN_USING_ENCODER is being used, so the power can be positive or negative
        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);

        //Target forwards or backwards is dictated by the sign of motor power variable
        leftFront.setTargetPosition(leftFront.getCurrentPosition() + distance * (int) Math.signum(leftFrontPower));
        leftBack.setTargetPosition(leftBack.getCurrentPosition() + distance * (int) Math.signum(leftBackPower));
        rightFront.setTargetPosition(rightFront.getCurrentPosition() +  distance * (int) Math.signum(rightFrontPower));
        rightBack.setTargetPosition(rightBack.getCurrentPosition() +  distance * (int) Math.signum(rightBackPower));

        //The target will be reached when the motors that have a power of 1 have reached their targets
        //For example, if moving 45 degrees, only LF and RB targets matter
        //This should still be the case even if the other wheels' power is not zero
        while(
                (
                    //|| can be used in this case because there is no RUN_TO_POSITION - the robot should not turn at the end
                    leftFront.getPower() == 1 && Math.abs(leftFront.getCurrentPosition() - leftFront.getTargetPosition()) < ENCODER_THRESHOLD ||
                    leftBack.getPower() == 1 && Math.abs(leftBack.getCurrentPosition() - leftBack.getTargetPosition()) < ENCODER_THRESHOLD ||
                    rightFront.getPower() == 1 && Math.abs(rightFront.getCurrentPosition() - rightFront.getTargetPosition()) < ENCODER_THRESHOLD ||
                    rightBack.getPower() == 1 && Math.abs(rightBack.getCurrentPosition() - rightBack.getTargetPosition()) < ENCODER_THRESHOLD
                ) && opModeIsActive()) {
            //Intentionally left empty
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        sleep(300);
    }
}
