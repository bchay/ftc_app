package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
import java.util.Date;
import java.util.Iterator;

class TaskData {
    long startTime;
    int delay;
    ThreadTaskInterface task;

    TaskData(int delay, ThreadTaskInterface task) {
        this.startTime = new Date().getTime();
        this.delay = delay;
        this.task = task;
    }

    static void executeThreads(ArrayList<TaskData> pendingTasks, LinearOpMode opMode) {
        Iterator<TaskData> iterator = pendingTasks.iterator();
        while (iterator.hasNext()) {
            if(!opMode.opModeIsActive()) break;
            TaskData task = iterator.next();

            if(new Date().getTime() - task.startTime > task.delay) {
                task.task.runTask();
                iterator.remove();
            }
        }
    }
}