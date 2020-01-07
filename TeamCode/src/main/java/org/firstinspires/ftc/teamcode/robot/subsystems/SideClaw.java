package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.*;

public class SideClaw {
    public Servo clawBar;
    public Servo clawClamp;

    public SideClaw(Servo clawBar, Servo clawClamp) {
        this.clawBar = clawBar;
        this.clawClamp = clawClamp;
    }
    public void openClaw() {
        clawClamp.setPosition(SIDE_CLAW_OPEN_POS);
    }
    public void closeClaw() {
        clawClamp.setPosition(SIDE_CLAW_CLOSE_POS);
    }
    public void hideClaw() {
        clawClamp.setPosition(SIDE_CLAW_IN_POS);
    }
    public void barUp() { clawBar.setPosition(SIDE_BAR_UP_POS); }
    public void barMid() { clawBar.setPosition(SIDE_BAR_MID_POS); }
    public void barDown() { clawBar.setPosition(SIDE_BAR_DOWN_POS); }
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
