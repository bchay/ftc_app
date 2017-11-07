package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Relic Recovery Teleop")
public class RelicRecoveryTeleop extends OpModeBase {
    private boolean xPreviousState = false; //Toggle grabbers
    private boolean bPreviousState = false; //Toggle grabbers
    private boolean slowMode = false;

    public void runOpMode() {
        super.runOpMode();

        motorLeft1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //PID not necessary, motors slower in RUN_USING_ENCODERS
        motorLeft2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while(opModeIsActive()) {
            motorLeft1.setPower(Math.pow(gamepad1.left_stick_y, 3) * (slowMode ? .5 : 1));
            motorLeft2.setPower(Math.pow(gamepad1.left_stick_y, 3) * (slowMode ? .5 : 1));
            motorRight1.setPower(Math.pow(gamepad1.right_stick_y, 3) * (slowMode ? .5 : 1));
            motorRight2.setPower(Math.pow(gamepad1.right_stick_y, 3) * (slowMode ? .5 : 1));

            if(gamepad2.x && !xPreviousState) { //X button has just been pressed - toggle servos

                //Need to check distance rather than == because floating point error may cause position to not be equal to constant
                if(Math.abs(leftGlyphGrabber.getPosition() - LEFT_GLYPH_GRABBR_CLOSED) < .1) leftGlyphGrabber.setPosition(LEFT_GLYPH_GRABBR_OPEN);
                else leftGlyphGrabber.setPosition(LEFT_GLYPH_GRABBR_CLOSED);

                if(Math.abs(rightGlyphGrabber.getPosition() - RIGHT_GLYPH_GRABBR_CLOSED) < .1) rightGlyphGrabber.setPosition(RIGHT_GLYPH_GRABBR_OPEN);
                else rightGlyphGrabber.setPosition(RIGHT_GLYPH_GRABBR_CLOSED);
            }

            if(gamepad1.b && !bPreviousState) { //X button has just been pressed - toggle servos
                slowMode = !slowMode;
            }

            xPreviousState = gamepad2.x;
            bPreviousState = gamepad1.b;

            //if(lift.getCurrentPosition() > 4500 && gamepad2.left_stick_y > 0) lift.setPower(gamepad2.left_stick_y); //Stop motor from hitting max
            if(lift.getCurrentPosition() < 0 && gamepad2.left_stick_y < 0) lift.setPower(gamepad2.left_stick_y); //Stop motor from hitting min;
            else if(lift.getCurrentPosition() >= 0/* && lift.getCurrentPosition() <= 4500*/) lift.setPower(gamepad2.left_stick_y); //Using touch sensor increases loop time from .1 ms to 10ms
            else lift.setPower(0);

            telemetry.addData("Left stick y", gamepad2.left_stick_y);
            telemetry.addData("Left Power", motorLeft1.getPower());
            telemetry.addData("Slow Mode", slowMode);
            telemetry.addData("Right Power", motorRight1.getPower());
            telemetry.addData("Left Grabber Position", leftGlyphGrabber.getPosition());
            telemetry.addData("Right Grabber Position", rightGlyphGrabber.getPosition());
            telemetry.addData("Lift Current Position", lift.getCurrentPosition());
            telemetry.addData("Voltage: ", this.hardwareMap.voltageSensor.iterator().next().getVoltage());
            telemetry.update();
        }
    }
}
