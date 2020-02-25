package com.millburnrobotics.skystone.auto.actions.sideclaw;

import com.millburnrobotics.skystone.auto.actions.Action;
import com.millburnrobotics.skystone.subsystems.Robot;

public class SideClawArmDownAction implements Action {
    @Override
    public void start() {
        Robot.getInstance().getSideClawLeft().armDown();
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
