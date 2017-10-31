package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Relic Recovery Teleop")
public class RelicRecoveryTeleop extends OpModeBase {
    private boolean xPreviousState = false; //Toggle grabbers
    private boolean yPreviousState = false; //Move lift up one position
    private boolean aPreviousState = false; //Move lift down one position
    private boolean dpadUpPreviousState = false; //Move lift down one position

    public void runOpMode() {
        super.runOpMode();

        motorLeft1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeft2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while(opModeIsActive()) {
            motorLeft1.setPower(gamepad1.left_stick_y);
            motorLeft2.setPower(gamepad1.left_stick_y);
            motorRight1.setPower(gamepad1.right_stick_y);
            motorRight2.setPower(gamepad1.right_stick_y);

            if(gamepad2.x && !xPreviousState) { //X button has just been pressed - toggle servos
                //Need to check distance rather than == because floating point error may cause position to not be equal to constant
                if(Math.abs(leftGlyphGrabber.getPosition() - LEFT_GLYPH_GRABBR_CLOSED) < .1) leftGlyphGrabber.setPosition(LEFT_GLYPH_GRABBR_OPEN);
                else leftGlyphGrabber.setPosition(LEFT_GLYPH_GRABBR_CLOSED);

                if(Math.abs(rightGlyphGrabber.getPosition() - RIGHT_GLYPH_GRABBR_CLOSED) < .1) rightGlyphGrabber.setPosition(RIGHT_GLYPH_GRABBR_OPEN);
                else rightGlyphGrabber.setPosition(RIGHT_GLYPH_GRABBR_CLOSED);
            }

            if(gamepad2.dpad_up && !dpadUpPreviousState) { //X button has just been pressed - toggle servos
                //Need to check distance rather than == because floating point error may cause position to not be equal to constant
                if(Math.abs(balancingStonePresser.getPosition() - BALANCING_STONE_PRESSER_IN) < .1) balancingStonePresser.setPosition(BALANCING_STONE_PRESSER_OUT);
                else balancingStonePresser.setPosition(BALANCING_STONE_PRESSER_IN);
            }

            //lift.setTargetPosition(gamepad2.left_stick_y > 0 ? lift.getTargetPosition() + 30 : lift.getTargetPosition() - 30);

            if(gamepad2.y && !yPreviousState) { //Y button has just been pressed - increase lift position
                if(currentLiftPosition < 2) { //Position 4 is max, don't set lift target to position 4
                    currentLiftPosition++;
                }
                lift.setTargetPosition(liftPositions[currentLiftPosition]);
            }

            if(gamepad2.a && !aPreviousState) { //X button has just been pressed - decrease lift position
                if(currentLiftPosition > 1) {
                    currentLiftPosition--;
                }
                lift.setTargetPosition(liftPositions[currentLiftPosition]);
            }

            if(lift.isBusy()) lift.setPower(1);
            else lift.setPower(0);

            xPreviousState = gamepad2.x;
            yPreviousState = gamepad2.y;
            aPreviousState = gamepad2.a;
            dpadUpPreviousState = gamepad1.dpad_up;

            telemetry.addData("Left Power", motorLeft1.getPower());
            telemetry.addData("Right Power", motorRight1.getPower());
            telemetry.addData("Left Grabber Position", leftGlyphGrabber.getPosition());
            telemetry.addData("Right Grabber Position", rightGlyphGrabber.getPosition());
            telemetry.addData("Lift Current Position", lift.getCurrentPosition());
            telemetry.addData("Lift Target Position", lift.getCurrentPosition());
            telemetry.addData("Voltage: ", this.hardwareMap.voltageSensor.iterator().next().getVoltage());
            telemetry.update();
        }
    }
}
