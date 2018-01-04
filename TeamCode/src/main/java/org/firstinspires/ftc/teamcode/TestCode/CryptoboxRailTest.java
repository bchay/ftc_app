package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.OpModeBase;
import org.firstinspires.ftc.teamcode.RelicRecoveryAutonomous;

@Autonomous(name = "Cryptobox Rail Test", group = "Test Code")
public class CryptoboxRailTest extends LinearOpMode {

    //Motors
    DcMotor motorLeft1;
    DcMotor motorLeft2;
    DcMotor motorRight1;
    DcMotor motorRight2;

    ModernRoboticsI2cRangeSensor range;


    public void runOpMode() {

        motorLeft1 = hardwareMap.dcMotor.get("left 1");
        motorLeft2 = hardwareMap.dcMotor.get("left 2");
        motorRight1 = hardwareMap.dcMotor.get("right 1");
        motorRight2 = hardwareMap.dcMotor.get("right 2");

        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");


        motorLeft1.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLeft2.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRight1.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRight2.setDirection(DcMotorSimple.Direction.REVERSE);

        motorLeft1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeft2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLeft1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorLeft2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorRight1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorRight2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();
        moveUntilCryptoboxRail(true, 2);
    }

    private void moveUntilCryptoboxRail(boolean isForward, int railNumber) {
        motorLeft1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeft2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLeft1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorLeft2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorRight1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorRight2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        int initialEncoderPosition = motorLeft1.getCurrentPosition();

        for (int i = 0; i < railNumber; i++) { //Move until railNumber of cryptobox rails have been detected
            if (!opModeIsActive()) break; //For loop cannot have opModeIsActive() check
            double previousDistance = range.getDistance(DistanceUnit.CM);

            //Loop while sensor does not sense a close object (rail)
            //Robot must move at least 500 encoder ticks
            //Cryptobox rail is 10cm / 4in
            while((previousDistance - range.getDistance(DistanceUnit.CM)) < 4 && opModeIsActive()) {
                double currentDistance = range.getDistance(DistanceUnit.CM);
                telemetry.addData("Range", currentDistance);
                telemetry.addData("Previous distance", previousDistance);
                telemetry.addData("Difference", (previousDistance - currentDistance));
                telemetry.addData("Distance travelled", Math.abs(motorLeft1.getCurrentPosition() - initialEncoderPosition));
                telemetry.addData("Rail", i);
                telemetry.update();

                //previousDistance = range.getDistance(DistanceUnit.CM);
                //if(currentDistance - previousDistance < 4) previousDistance = currentDistance;
                previousDistance = currentDistance;
            }

            telemetry.addData("Rail Reached", "");
            telemetry.update();

            if (!opModeIsActive()) break; //For loop cannot have opModeIsActive() check
        }
    }
}
