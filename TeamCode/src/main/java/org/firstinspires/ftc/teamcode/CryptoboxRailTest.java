package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Cryptobox Rail Test", group = "Test Code")
public class CryptoboxRailTest extends OpModeBase {
    public void runOpMode() {
        super.runOpMode(RelicRecoveryAutonomous.class);

        waitForStart();
        moveUntilCryptoboxRail(true, 2);
    }
}
