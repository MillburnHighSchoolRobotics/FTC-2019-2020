package com.millburnrobotics.skystone.auto.actions.chainbar;

import com.millburnrobotics.skystone.Robot;
import com.millburnrobotics.skystone.auto.actions.Action;

public class ChainBarOutAction implements Action {
    @Override
    public void start() {
        Robot.getInstance().getChainBar().chainBarOut();
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
