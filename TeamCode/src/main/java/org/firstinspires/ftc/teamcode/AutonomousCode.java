package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.graphics.Color;
import android.preference.PreferenceManager;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "Autonomous", group = "Main Code")
public class AutonomousCode extends OpModeBase {
    public void runOpMode() {
        super.init();
        waitForStart();
        sleep(delay);

        //Robot begins centered at third tile away from corner vortex wall
        if(location.equals("Close")) {
            move(12, moveSpeed);
            turn(48, moveDirection, turnSpeed);
            move(25, 1); //Approach white line
            driveToWhiteLine(.4);
            move(3.5, .5); //Position robot so that servos are in correct location
            turn(42, moveDirection, turnSpeed);
            move(3.5, moveSpeed);

            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsv);
            String beaconColor = getColorName(hsv);

            if(beaconColor.equals(allianceColor)) {
                buttonPresserRight.setPosition(BUTTON_PRESSER_RIGHT_OUT);
            } else {
                buttonPresserLeft.setPosition(BUTTON_PRESSER_LEFT_OUT);
            }

            sleep(400);
            move(15, .65); //Push beacon

            buttonPresserRight.setPosition(BUTTON_PRESSER_RIGHT_IN);
            buttonPresserLeft.setPosition(BUTTON_PRESSER_LEFT_IN);

            move(-5, moveSpeed); //Back away from wall
            move(50, moveSpeed);
            turn(90, moveDirection.next(), turnSpeed);
            move(40, moveSpeed); //Drive to second beacon
            driveToWhiteLine(.4);
            move(4, .5);
            turn(90, moveDirection, turnSpeed);

            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsv);
            beaconColor = getColorName(hsv);

            if(beaconColor.equals(allianceColor)) {
                buttonPresserRight.setPosition(BUTTON_PRESSER_RIGHT_OUT);
            } else {
                buttonPresserLeft.setPosition(BUTTON_PRESSER_LEFT_OUT);
            }

            sleep(400);
            move(16, .65); //Push beacon
        }
    }
}