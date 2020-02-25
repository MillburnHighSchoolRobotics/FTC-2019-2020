package com.millburnrobotics.skystone.auto.actions.sideclaw;

import com.millburnrobotics.skystone.auto.actions.Action;
import com.millburnrobotics.skystone.subsystems.Robot;
import com.millburnrobotics.skystone.subsystems.SideClaw;

public class SideClawArmUpAction implements Action {
    private SideClaw.SideClawSide side;

    public SideClawArmUpAction(SideClaw.SideClawSide side) {
        this.side = side;
    }

    @Override
    public void start() {
        if (side == SideClaw.SideClawSide.LEFT){
            Robot.getInstance().getSideClawLeft().armUp();
        } else {
            Robot.getInstance().getSideClawRight().armUp();
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
