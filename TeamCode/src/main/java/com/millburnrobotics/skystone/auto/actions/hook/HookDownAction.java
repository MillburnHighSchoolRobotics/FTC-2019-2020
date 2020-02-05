package com.millburnrobotics.skystone.auto.actions.hook;

import com.millburnrobotics.skystone.auto.actions.Action;
import com.millburnrobotics.skystone.subsystems.Robot;

public class HookDownAction implements Action {
    @Override
    public void start() {
        Robot.getInstance().getHook().hookDown();
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