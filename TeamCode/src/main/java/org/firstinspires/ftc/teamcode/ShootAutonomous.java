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

@Autonomous(name = "Shooter Autonomous", group = "Main Code")
public class ShootAutonomous extends OpModeBase {
    public void runOpMode() {
        super.init();
        waitForStart();
        sleep(delay);


        //Robot begins third tile away from corner vortex wall, wheels touching next full tile next to vortex
        if(location.equals("Close")) {
            shoot();

            move(12, moveSpeed);
            turn(48, moveDirection, turnSpeed);
            move(25, 1); //Approach white line
            driveToWhiteLine(.4);
            move(3.5, .4); //Position robot so that servos are in correct location
            turn(42, moveDirection, turnSpeed);
            move(3.5, moveSpeed);

            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsv);
            String beaconColor = getColorName(hsv);

            if(beaconColor.equals(allianceColor)) {
                buttonPresserRight.setPosition(BUTTON_PRESSER_RIGHT_OUT);
            } else {
                buttonPresserLeft.setPosition(BUTTON_PRESSER_LEFT_OUT);
            }

            sleep(1000);
            move(15, .45); //Push beacon

            buttonPresserRight.setPosition(BUTTON_PRESSER_RIGHT_IN);
            buttonPresserLeft.setPosition(BUTTON_PRESSER_LEFT_IN);
        }
    }
}