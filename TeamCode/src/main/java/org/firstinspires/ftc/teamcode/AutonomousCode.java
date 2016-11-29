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
        super.runOpMode();
        waitForStart();
        sleep(delay);

        //Robot begins centered at third tile away from corner vortex wall
        if(location.equals("Close")) {
            followLine(.3);
            /*
            //First Beacon
            move(12, moveSpeed);
            turn(50, moveDirection, turnSpeed, -1); //Adjust twice
            move(30, 1); //Approach white line
            driveToWhiteLine(.2);
            move(.1, .3); //Center robot to read color and push beacon

            turn(40, moveDirection, turnSpeed);
            moveUntilDistance(10, .2); //Approach to within ten inches of beacon to read color

            String beaconColor = getColorName();
            if(beaconColor.equals(allianceColor)) {
                buttonPresser.setPosition(BUTTON_PRESSER_LEFT);
            } else if (!beaconColor.equals("Undefined")) { //Alliance color is on the right
                buttonPresser.setPosition(BUTTON_PRESSER_RIGHT);
            } else return; //Beacon color is undefined, something went wrong
            sleep(400);
            move(10, .3); //Push beacon
*/
            //Second Beacon
            //moveUntilDistance(15, -.1);
            //turn(90, moveDirection.next(), turnSpeed);
            //move(43, moveSpeed);

/*
            move(-5, moveSpeed); //Back away from wall
            move(50, moveSpeed);
            turn(90, moveDirection.next(), turnSpeed);
            move(40, moveSpeed); //Drive to second beacon
            driveToWhiteLine(.4);
            move(4, .5);
            turn(90, moveDirection, turnSpeed);

            beaconColor = getColorName();

            if(beaconColor.equals(allianceColor)) {
                buttonPresser.setPosition(BUTTON_PRESSER_LEFT);
            } else {
                buttonPresser.setPosition(BUTTON_PRESSER_RIGHT);
            }

            sleep(400);
            move(16, .65); //Push beacon
            */
        }
    }
}