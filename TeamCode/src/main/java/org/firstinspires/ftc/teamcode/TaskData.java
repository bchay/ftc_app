package org.firstinspires.ftc.teamcode;

import java.util.Date;

public class TaskData {
    long startTime;
    int delay;
    ThreadTaskInterface task;

    TaskData(int delay, ThreadTaskInterface task) {
        this.startTime = new Date().getTime();
        this.delay = delay;
        this.task = task;
    }
}
