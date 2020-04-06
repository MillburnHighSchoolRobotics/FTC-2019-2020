package com.millburnrobotics.skystone.auto.actions.hook;

import com.millburnrobotics.skystone.Robot;
import com.millburnrobotics.skystone.auto.actions.Action;

public class HookMidAction implements Action {
    @Override
    public void start() {
        Robot.getInstance().getHook().hookMid();
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
