package com.millburnrobotics.skystone.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static com.millburnrobotics.skystone.Constants.SideClawConstants.SIDE_ARM_DOWN_POS;
import static com.millburnrobotics.skystone.Constants.SideClawConstants.SIDE_ARM_INIT_POS;
import static com.millburnrobotics.skystone.Constants.SideClawConstants.SIDE_ARM_MID_POS;
import static com.millburnrobotics.skystone.Constants.SideClawConstants.SIDE_ARM_UP_POS;
import static com.millburnrobotics.skystone.Constants.SideClawConstants.SIDE_CLAW_CLOSE_POS;
import static com.millburnrobotics.skystone.Constants.SideClawConstants.SIDE_CLAW_OPEN_POS;

public class SideClaw extends Subsystem {
    double currentArmPos;
    double currentClawPos;
    ElapsedTime changeSideArm = new ElapsedTime();
    ElapsedTime changeSideClaw = new ElapsedTime();
    @Override
    public void init(boolean auto) {
        if (auto) {
            armInit();
            clawOpen();
        } else {
            armUp();
            clawClose();
            changeSideArm.reset();
            changeSideClaw.reset();
        }
    }

    @Override
    public void outputToTelemetry(Telemetry telemetry) {
        telemetry.addData("CurrentSideArmPosition", currentArmPos);
        telemetry.addData("CurrentSideClawPosition", currentClawPos);
    }

    @Override
    public void update() {

    }

    public void armUp() {
        setArmPosition(SIDE_ARM_UP_POS);
    }
    public void armMid() {
        setArmPosition(SIDE_ARM_MID_POS);
    }
    public void armDown() {
        setArmPosition(SIDE_ARM_DOWN_POS);
    }
    public void armInit() {
        setArmPosition(SIDE_ARM_INIT_POS);
    }
    public void setArmPosition(double pos) {
        currentArmPos = pos;
        changeSideArm.reset();
        Robot.getInstance().sideClawArm.setPosition(currentArmPos);
    }
    public double getArmPosition() {
        return currentArmPos;
    }
    public boolean canToggleSideArm() {
        return changeSideArm.milliseconds() > 25;
    }

    public void clawClose() {
        setClawPosition(SIDE_CLAW_CLOSE_POS);
    }
    public void clawOpen() {
        setClawPosition(SIDE_CLAW_OPEN_POS);
    }
    public void setClawPosition(double pos) {
        currentClawPos = pos;
        changeSideClaw.reset();
        Robot.getInstance().sideClawClaw.setPosition(currentClawPos);
    }
    public double getClawPosition() {
        return currentClawPos;
    }
    public boolean canToggleSideClaw() {
        return changeSideClaw.milliseconds() > 25;
    }
}
