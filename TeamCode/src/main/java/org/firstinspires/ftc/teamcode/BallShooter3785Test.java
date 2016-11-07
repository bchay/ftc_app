package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class BallShooter3785Test extends OpMode {
    DcMotor motor;
    double motorPower = 0;
    public void init() {
        motor = hardwareMap.dcMotor.get("motor");
    }
    public void loop() {
        if(gamepad1.a) motorPower += .01;
        if(gamepad1.b) motorPower -= .01;
    }
}
