package com.millburnrobotics.skystone.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import static com.millburnrobotics.skystone.Constants.HookConstants.LEFT_HOOK_DOWN_POS;
import static com.millburnrobotics.skystone.Constants.HookConstants.LEFT_HOOK_UP_POS;
import static com.millburnrobotics.skystone.Constants.HookConstants.RIGHT_HOOK_DOWN_POS;
import static com.millburnrobotics.skystone.Constants.HookConstants.RIGHT_HOOK_UP_POS;

public class Hook {
    Servo hookLeft, hookRight;
    public Hook(Servo foundationHookLeft, Servo foundationHookRight) {
        this.hookLeft = foundationHookLeft;
        this.hookRight = foundationHookRight;
    }
    public void hookDown() {
        hookLeft.setPosition(LEFT_HOOK_DOWN_POS);
        hookRight.setPosition(RIGHT_HOOK_DOWN_POS);
    }
    public void hookUp() {
        hookLeft.setPosition(LEFT_HOOK_UP_POS);
        hookRight.setPosition(RIGHT_HOOK_UP_POS);
    }
}
