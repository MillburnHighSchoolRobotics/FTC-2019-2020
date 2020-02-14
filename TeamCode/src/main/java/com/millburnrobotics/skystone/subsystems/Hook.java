package com.millburnrobotics.skystone.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static com.millburnrobotics.skystone.Constants.HookConstants.LEFT_HOOK_DOWN_POS;
import static com.millburnrobotics.skystone.Constants.HookConstants.LEFT_HOOK_UP_POS;
import static com.millburnrobotics.skystone.Constants.HookConstants.RIGHT_HOOK_DOWN_POS;
import static com.millburnrobotics.skystone.Constants.HookConstants.RIGHT_HOOK_UP_POS;

public class Hook extends Subsystem {
    public enum HookState {
        HOOK_DOWN,
        HOOK_UP
    }
    private HookState state;
    private ElapsedTime toggleHook = new ElapsedTime();

    @Override
    public void init(boolean auto) {
//        hookUp();
    }

    @Override
    public void outputToTelemetry(Telemetry telemetry) {
//        telemetry.addData("HookState", state.name());
    }

    @Override
    public void update() {

    }

    public void hookDown() {
        setHookPosition(LEFT_HOOK_DOWN_POS, RIGHT_HOOK_DOWN_POS);
        state = HookState.HOOK_DOWN;
    }
    public void hookUp() {
        setHookPosition(LEFT_HOOK_UP_POS, RIGHT_HOOK_UP_POS);
        state = HookState.HOOK_UP;
    }
    public void setHookPosition(double leftPos, double rightPos) {
        Robot.getInstance().hookL.setPosition(leftPos);
        Robot.getInstance().hookR.setPosition(rightPos);
        toggleHook.reset();
    }
    public boolean toggleHook() {
        return (toggleHook.milliseconds() > 250);
    }
    public HookState getState() {
        return state;
    }
}
