package com.millburnrobotics.skystone.auto.actions.sideclaw;

import com.millburnrobotics.skystone.auto.actions.Action;
import com.millburnrobotics.skystone.subsystems.Robot;
import com.millburnrobotics.skystone.subsystems.SideClaw;

public class SideClawArmMidAction implements Action {
    private SideClaw.SideClawSide side;

    public SideClawArmMidAction(SideClaw.SideClawSide side) {
        this.side = side;
    }

    @Override
    public void start() {
        if (side == SideClaw.SideClawSide.LEFT){
            Robot.getInstance().getSideClawLeft().armMid();
        } else {
            Robot.getInstance().getSideClawRight().armMid();
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
