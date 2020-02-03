package com.millburnrobotics.skystone.subsystems.old;

import com.millburnrobotics.skystone.Constants;
import com.qualcomm.robotcore.hardware.Servo;

public class SideClaw {
    public Servo clawBar;
    public Servo clawClamp;

    public SideClaw(Servo clawBar, Servo clawClamp) {
        this.clawBar = clawBar;
        this.clawClamp = clawClamp;
    }
    public void openClaw() {
        clawClamp.setPosition(Constants.SideClawConstants.SIDE_CLAW_OPEN_POS);
    }
    public void closeClaw() {
        clawClamp.setPosition(Constants.SideClawConstants.SIDE_CLAW_CLOSE_POS);
    }
    public void hideClaw() {
        clawClamp.setPosition(Constants.SideClawConstants.SIDE_CLAW_IN_POS);
    }
    public void initClaw() {
        clawClamp.setPosition(Constants.SideClawConstants.SIDE_CLAW_INIT_POS);
    }
    public void barUp() { clawBar.setPosition(Constants.SideClawConstants.SIDE_BAR_UP_POS); }
    public void barMid() { clawBar.setPosition(Constants.SideClawConstants.SIDE_BAR_MID_POS); }
    public void barDown() { clawBar.setPosition(Constants.SideClawConstants.SIDE_BAR_DOWN_POS); }
    public void grab() {
        openClaw();
        barDown();
        closeClaw();
        barUp();
    }
    public void release() {
        barDown();
        openClaw();
        barUp();
        closeClaw();
    }
}