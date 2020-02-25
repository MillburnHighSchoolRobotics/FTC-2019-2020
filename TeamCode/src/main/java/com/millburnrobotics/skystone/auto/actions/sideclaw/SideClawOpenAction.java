package com.millburnrobotics.skystone.auto.actions.sideclaw;

import com.millburnrobotics.skystone.auto.actions.Action;
import com.millburnrobotics.skystone.subsystems.Robot;
import com.millburnrobotics.skystone.subsystems.SideClaw;

public class SideClawOpenAction implements Action {
    private SideClaw.SideClawSide side;

    public SideClawOpenAction(SideClaw.SideClawSide side) {
        this.side = side;
    }

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
