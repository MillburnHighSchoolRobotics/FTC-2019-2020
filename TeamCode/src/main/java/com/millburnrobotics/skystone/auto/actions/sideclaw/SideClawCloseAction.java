package com.millburnrobotics.skystone.auto.actions.sideclaw;

import com.millburnrobotics.skystone.auto.actions.Action;
import com.millburnrobotics.skystone.subsystems.Robot;
import com.millburnrobotics.skystone.subsystems.SideClaw;

public class SideClawCloseAction implements Action {
    private SideClaw.SideClawSide side;

    public SideClawCloseAction(SideClaw.SideClawSide side) {
        this.side = side;
    }

    @Override
    public void start() {
        if (side == SideClaw.SideClawSide.LEFT){
            Robot.getInstance().getSideClawLeft().clawClose();
        } else {
            Robot.getInstance().getSideClawRight().clawClose();
        }
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
