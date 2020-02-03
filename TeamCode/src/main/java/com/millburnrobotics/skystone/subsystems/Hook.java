package com.millburnrobotics.skystone.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static com.millburnrobotics.skystone.Constants.HookConstants.LEFT_HOOK_DOWN_POS;
import static com.millburnrobotics.skystone.Constants.HookConstants.LEFT_HOOK_UP_POS;
import static com.millburnrobotics.skystone.Constants.HookConstants.RIGHT_HOOK_DOWN_POS;
import static com.millburnrobotics.skystone.Constants.HookConstants.RIGHT_HOOK_UP_POS;

public class Hook extends Subsystem {
    @Override
    public void init(boolean auto) {
    }

    @Override
    public void outputToTelemetry(Telemetry telemetry) {

    }

    @Override
    public void update() {

    }

    public void hookDown() {
        setHookPosition(LEFT_HOOK_DOWN_POS, RIGHT_HOOK_DOWN_POS);
    }
    public void hookUp() {
        setHookPosition(LEFT_HOOK_UP_POS, RIGHT_HOOK_UP_POS);
    }
    public void setHookPosition(double leftPos, double rightPos) {
        Robot.getInstance().hookL.setPosition(leftPos);
        Robot.getInstance().hookR.setPosition(rightPos);
    }
}
