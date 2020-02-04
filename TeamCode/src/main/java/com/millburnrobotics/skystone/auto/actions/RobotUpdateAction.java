package com.millburnrobotics.skystone.auto.actions;

import com.millburnrobotics.skystone.subsystems.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotUpdateAction implements Action {
    ElapsedTime timer;
    public RobotUpdateAction(ElapsedTime timer) {
        this.timer = timer;
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void update() {
        Robot.getInstance().update();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void done() {

    }
}
