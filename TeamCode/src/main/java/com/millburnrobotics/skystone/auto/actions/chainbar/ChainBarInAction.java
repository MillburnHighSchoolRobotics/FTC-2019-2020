package com.millburnrobotics.skystone.auto.actions.chainbar;

import com.millburnrobotics.skystone.Robot;
import com.millburnrobotics.skystone.auto.actions.Action;

public class ChainBarInAction implements Action {
    @Override
    public void start() {
        Robot.getInstance().getChainBar().chainBarIn();
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
