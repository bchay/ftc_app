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

@Autonomous(name = "One Beacon", group = "Main Code")
public class OneBeacon extends OpModeBase {
    public void runOpMode() {
        super.runOpMode();
        waitForStart();
        sleep(delay);

        //Robot begins centered at third tile away from corner vortex wall
        moveSpeed = .6;
        turnSpeed = .15;
        slowdownMin = .1;

        //First Beacon
        move(12, moveSpeed);
        turn(48, moveDirection, turnSpeed, -2); //Adjust twice
        move(34, moveSpeed); //Approach white line
        driveToWhiteLine(.1);
        move(1, .3);
        turn(37, moveDirection, turnSpeed, -1);

        telemetry.addData("Beacon Presser location", buttonPresser.getPosition());
        telemetry.update();

        moveUntilDistance(16, .1);

        followLine(5, .2); //Approach to within ten inches of beacon to read color

        String beaconColor = getColorName();
        if (beaconColor.equals(allianceColor)) {
            buttonPresser.setPosition(BUTTON_PRESSER_LEFT);
        } else if (!beaconColor.equals("Undefined")) { //Alliance color is on the right
            buttonPresser.setPosition(BUTTON_PRESSER_RIGHT);
        }

        sleep(1500);
        move(40, 1); //Push beacon
    }
}