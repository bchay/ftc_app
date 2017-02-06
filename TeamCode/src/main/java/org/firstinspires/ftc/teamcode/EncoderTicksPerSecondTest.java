package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Ticks per Second Test", group = "Test Code")
public class EncoderTicksPerSecondTest extends OpMode {
    private DcMotor motor;

    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private double previousTime = 0;
    private double previousTicks = 0;

    private double totalValue = 0;
    private double numRecorded = 0;

    public void init() {
        motor = hardwareMap.dcMotor.get("motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void start() {
        timer.reset();

    }

    public void loop() {
        motor.setPower(.5);

        double elapsedTime = timer.seconds() - previousTime;
        double elapsedTicks = motor.getCurrentPosition() - previousTicks;

        previousTime = timer.seconds();
        previousTicks = motor.getCurrentPosition();
        double ticksPerSecond = (elapsedTicks / elapsedTime);

        totalValue += ticksPerSecond;
        numRecorded++;

        telemetry.addData("Ticks per second", elapsedTicks / elapsedTime);
        telemetry.addData("Average", totalValue / numRecorded);
        telemetry.addData("Voltage: ", this.hardwareMap.voltageSensor.iterator().next().getVoltage());
    }
}

/*
2000 ticks at 12.3 volts
2000 ticks at 12.9 volts
2000 at 13.8 volts
*/