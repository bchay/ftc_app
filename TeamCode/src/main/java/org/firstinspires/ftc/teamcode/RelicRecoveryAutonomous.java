package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Relic Recovery Autonomous")
public class RelicRecoveryAutonomous extends OpModeBase { //MecanumTeleop is a LinearOpMode so it can extend the same base class as autonomous

    public void runOpMode() {
        super.runOpMode();
        waitForStart();

        move(24);

        /*

        arm.setPosition(ARM_OUT);
        sleep(1000);

        while(getColor() == 0 && opModeIsActive()) {
            telemetry.addData("Color", "Unknown");
            telemetry.update();
        }

        //Color sensor reads right jewel
        if(getColor() == Color.RED && allianceColor.equals("Red")) move(5);
        else if(getColor() == Color.RED && allianceColor.equals("Blue")) move(-5);
        else if(getColor() == Color.BLUE && allianceColor.equals("Blue")) move(5);
        else if(getColor() == Color.BLUE && allianceColor.equals("Red")) move(-5);
    */
    }

}