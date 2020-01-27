package com.millburnrobotics.skystone.auto.actions;

public abstract class ThreadAction implements Action {
    @Override
    public void start() {
//        if(opModeIsActive() && !isStopRequested()) {
//            new Thread(runnable).start();
//        }
//        runAction(action);
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void done() {}
}