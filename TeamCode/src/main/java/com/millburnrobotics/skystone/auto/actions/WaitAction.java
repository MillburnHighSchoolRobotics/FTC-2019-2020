package com.millburnrobotics.skystone.auto.actions;

import com.qualcomm.robotcore.util.ElapsedTime;

public class WaitAction implements Action {
    private final double duration;
    private ElapsedTime timer;

    public WaitAction(double duration) {
        this.duration = duration;
    }

    @Override
    public void start() {
        timer = new ElapsedTime();
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return timer.milliseconds() > duration;
    }

    @Override
    public void done() {}
}