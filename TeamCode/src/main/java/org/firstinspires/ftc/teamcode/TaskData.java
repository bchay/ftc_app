package org.firstinspires.ftc.teamcode;

import java.util.Date;

public class TaskData {
    public long startTime;
    public int delay;
    public ThreadTaskInterface task;

    public TaskData(int delay, ThreadTaskInterface task) {
        this.startTime = new Date().getTime();
        this.delay = delay;
        this.task = task;
    }
}
