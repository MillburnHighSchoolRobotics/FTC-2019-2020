package com.millburnrobotics.skystone.auto.actions.sideclaw;

import com.millburnrobotics.skystone.Robot;
import com.millburnrobotics.skystone.auto.actions.Action;

public class SideClawCloseAction implements Action {

    public SideClawCloseAction() {
    }

    @Override
    public void start() {
        Robot.getInstance().getSideClaw().clawClose();
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
