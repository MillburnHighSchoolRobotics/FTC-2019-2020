package com.millburnrobotics.skystone.auto.actions.chainbar;

import com.millburnrobotics.skystone.auto.actions.Action;
import com.millburnrobotics.skystone.Robot;

public class ChainBarUpAction implements Action {
    @Override
    public void start() {
        Robot.getInstance().getChainBar().chainBarUp();
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
