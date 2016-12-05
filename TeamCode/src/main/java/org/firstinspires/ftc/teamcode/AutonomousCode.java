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
public class AutonomousCode extends OpModeBase { //Robot begins angled
    public void runOpMode() {
        super.runOpMode();
        waitForStart();
        sleep(delay);

        //First Beacon
            /*
            move(12, moveSpeed);
            turn(46, moveDirection, turnSpeed, -1); //Adjust twice
            */

        move(56, 1); //Approach white line
        driveToWhiteLine(.1);
        move(1, .4);
        turn(44, moveDirection, turnSpeed); //Point toward beacon

        moveUntilDistance(16, .4); //Back up to make line follower more accurate
        followLine(7, .3); //Approach to within ten inches of beacon to read color

        String beaconColor = getColorName();
        if (beaconColor.equals(allianceColor)) {
            buttonPresser.setPosition(BUTTON_PRESSER_LEFT);
        } else if (!beaconColor.equals("Undefined")) { //Alliance color is on the right
            buttonPresser.setPosition(BUTTON_PRESSER_RIGHT);
        } //If beacon color is "Undefined" the robot will still approach beacon in hopes of hitting the second.

        sleep(600);

        move(5, .4); //Push beacon

        //Second Beacon
        buttonPresser.setPosition(BUTTON_PRESSER_NEUTRAL);

        moveUntilDistance(7, .4); //Back up

        turn(90, moveDirection.next(), turnSpeed, -1); //Turn towards second beacon
        move(35, moveSpeed); //Move to second beacon location

        driveToWhiteLine(.1);
        turn(90, moveDirection, turnSpeed, -1);

        move(11, 1); //Move past white line to center ODS to side

        moveUntilDistance(14, .4); //Back up to make line follower more accurate

        followLine(7, .3); //Approach to within ten inches of beacon to read color

        beaconColor = getColorName();
        if (beaconColor.equals(allianceColor)) {
            buttonPresser.setPosition(BUTTON_PRESSER_LEFT);
        } else if (!beaconColor.equals("Undefined")) { //Alliance color is on the right
            buttonPresser.setPosition(BUTTON_PRESSER_RIGHT);
        } else return; //Beacon color is undefined, something went wrong

        sleep(600);
        move(15, .35); //Push beacon
    }
}