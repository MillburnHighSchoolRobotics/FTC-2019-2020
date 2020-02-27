package com.millburnrobotics.skystone.auto.actions.sideclaw;

import com.millburnrobotics.skystone.auto.actions.Action;
import com.millburnrobotics.skystone.Robot;

public class SideClawOpenAction implements Action {
    @Override
    public void start() {
        Robot.getInstance().getSideClaw().clawOpen();
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
