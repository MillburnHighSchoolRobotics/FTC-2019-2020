package com.millburnrobotics.skystone.auto.actions.drive;

import com.millburnrobotics.skystone.auto.actions.Action;
import com.millburnrobotics.skystone.subsystems.Robot;

public class HookUpAction implements Action {
    @Override
    public void start() {
        Robot.getInstance().getHook().hookUp();
    }

    @Override
    public void update() { }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void done() { }
}