package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.Date;
import java.util.Iterator;

@TeleOp(name = "Intake Align Test", group = "Test Code")
public class IntakeAutoAlignTest extends OpMode {
    private DcMotor leftIntake;
    private DcMotor rightIntake;

    private int previousLeftIntakeEncoder = 0;
    private int previousRightIntakeEncoder = 0;

    private double previousMotorPower = 0;

    private boolean checkMotors = true;

    private ArrayList<IntakeAutoAlignTest.TaskData> pendingTasks = new ArrayList<>();

    public void init() {
        leftIntake = hardwareMap.dcMotor.get("left intake");
        rightIntake = hardwareMap.dcMotor.get("right intake");

        leftIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void loop() {
        int leftIntakeEncoder = leftIntake.getCurrentPosition();
        int rightIntakeEncoder = rightIntake.getCurrentPosition();

        if(gamepad1.left_bumper) { //Suck glyphs in
            //If encoder values have changed, or if motor power was previously zero - ie. motors just started
            if(checkMotors && ((Math.abs(leftIntakeEncoder - previousLeftIntakeEncoder) > 0) && (Math.abs(rightIntakeEncoder - previousRightIntakeEncoder) > 0)) || previousMotorPower == 0) {
                leftIntake.setPower(-1);
                rightIntake.setPower(1);
            } else if(checkMotors) {
                leftIntake.setPower(.4); //Reverse one side of intake so that glyph aligns properly
                rightIntake.setPower(1);

                checkMotors = false;

                pendingTasks.add(new TaskData(400, new ThreadTaskInterface() {
                    @Override
                    public void runTask() {
                        checkMotors = true;
                    }
                }));
            }
        } else {
            leftIntake.setPower(0);
            rightIntake.setPower(0);
        }

        previousMotorPower = leftIntake.getPower();

        telemetry.addData("Check Motors", checkMotors);
        telemetry.addData("Left Encoder Delta", Math.abs(leftIntakeEncoder - previousLeftIntakeEncoder));
        telemetry.addData("Right Encoder Delta", Math.abs(rightIntakeEncoder - previousRightIntakeEncoder));
        telemetry.update();

        previousLeftIntakeEncoder = leftIntakeEncoder;
        previousRightIntakeEncoder = rightIntakeEncoder;

        executeThreads();
    }

    private class TaskData {
        private long startTime;
        private int delay;
        private IntakeAutoAlignTest.ThreadTaskInterface task;

        TaskData(int delay, IntakeAutoAlignTest.ThreadTaskInterface task) {
            this.startTime = new Date().getTime();
            this.delay = delay;
            this.task = task;
        }
    }

    private interface ThreadTaskInterface {
        void runTask();
    }

    private void executeThreads() {
        Iterator<IntakeAutoAlignTest.TaskData> iterator = pendingTasks.iterator();
        while (iterator.hasNext()) {
            IntakeAutoAlignTest.TaskData task = iterator.next();

            if(new Date().getTime() - task.startTime > task.delay) {
                task.task.runTask();
                iterator.remove();
            }
        }
    }
}