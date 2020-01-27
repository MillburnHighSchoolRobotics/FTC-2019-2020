package com.millburnrobotics.skystone.auto.actions;

public interface Action {
    void start();

    void update();

    boolean isFinished();

    void done();
}
