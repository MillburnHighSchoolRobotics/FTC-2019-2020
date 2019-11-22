package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.LEFT_HOOK_DOWN_POS;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.LEFT_HOOK_UP_POS;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.RIGHT_HOOK_DOWN_POS;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.RIGHT_HOOK_UP_POS;

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
