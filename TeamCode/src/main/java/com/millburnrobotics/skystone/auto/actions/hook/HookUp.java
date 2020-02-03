package com.millburnrobotics.skystone.auto.actions.hook;

import com.millburnrobotics.skystone.auto.actions.RunOnceAction;
import com.millburnrobotics.skystone.subsystems.Robot;

public class HookUp extends RunOnceAction {
    @Override
    public void runOnce() {
        Robot.getInstance().getHook().hookUp();
    }
}
