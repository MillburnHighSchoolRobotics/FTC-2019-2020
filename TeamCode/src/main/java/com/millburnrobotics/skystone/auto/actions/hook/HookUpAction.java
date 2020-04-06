package com.millburnrobotics.skystone.auto.actions.hook;

import com.millburnrobotics.skystone.Robot;
import com.millburnrobotics.skystone.auto.actions.Action;

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
