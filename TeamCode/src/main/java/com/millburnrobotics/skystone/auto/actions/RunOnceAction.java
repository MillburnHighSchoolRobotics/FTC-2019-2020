package com.millburnrobotics.skystone.auto.actions;

public abstract class RunOnceAction implements Action {
    @Override
    public void start() {
        runOnce();
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void done() {}

    public abstract void runOnce();
}