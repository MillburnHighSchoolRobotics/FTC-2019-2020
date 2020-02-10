package com.millburnrobotics.skystone.auto.actions.intake;

import com.millburnrobotics.skystone.auto.actions.Action;
import com.millburnrobotics.skystone.subsystems.Robot;

public class IntakeOutAction implements Action {
    @Override
    public void start() {
        Robot.getInstance().getIntake().intakeOut();
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
