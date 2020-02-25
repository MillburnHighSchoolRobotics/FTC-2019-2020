package com.millburnrobotics.skystone.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static com.millburnrobotics.skystone.Constants.SideClawConstants.*;

public class SideClaw extends Subsystem {
    public enum SideClawSide {
        LEFT,
        RIGHT
    }
    private SideClawSide side;
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
        telemetry.addData("CurrentSideArmPosition" + side.toString().charAt(0), currentArmPos);
        telemetry.addData("CurrentSideClawPosition" + side.toString().charAt(0), currentClawPos);
    }

    @Override
    public void update() {

    }

    public void setSide(SideClawSide side) {
        this.side = side;
    }

    public void armUp() {
        if (side == SideClawSide.LEFT) {
            setArmPosition(SIDE_ARM_L_UP_POS);
        } else {
            setArmPosition(SIDE_ARM_R_UP_POS);
        }
    }
    public void armMid() {
        if (side == SideClawSide.LEFT) {
            setArmPosition(SIDE_ARM_L_MID_POS);
        } else {
            setArmPosition(SIDE_ARM_R_MID_POS);
        }
    }
    public void armDown() {
        if (side == SideClawSide.LEFT) {
            setArmPosition(SIDE_ARM_L_DOWN_POS);
        } else {
            setArmPosition(SIDE_ARM_R_DOWN_POS);
        }
    }
    public void armInit() {
        if (side == SideClawSide.LEFT) {
            setArmPosition(SIDE_ARM_L_INIT_POS);
        } else {
            setArmPosition(SIDE_ARM_R_INIT_POS);
        }
    }
    public void setArmPosition(double pos) {
        currentArmPos = pos;
        changeSideArm.reset();
        Robot.getInstance().sideClawArmLeft.setPosition(currentArmPos);
    }
    public double getArmPosition() {
        return currentArmPos;
    }
    public boolean canToggleSideArm() {
        return changeSideArm.milliseconds() > 25;
    }

    public void clawClose() {
        if (side == SideClawSide.LEFT) {
            setArmPosition(SIDE_CLAW_L_CLOSE_POS);
        } else {
            setArmPosition(SIDE_CLAW_R_CLOSE_POS);
        }
    }
    public void clawOpen() {
        if (side == SideClawSide.LEFT) {
            setArmPosition(SIDE_CLAW_L_OPEN_POS);
        } else {
            setArmPosition(SIDE_CLAW_R_OPEN_POS);
        }
    }
    public void setClawPosition(double pos) {
        currentClawPos = pos;
        changeSideClaw.reset();
        if (side == SideClawSide.LEFT) {
            Robot.getInstance().sideClawClawLeft.setPosition(currentClawPos);
        } else {
            Robot.getInstance().sideClawClawRight.setPosition(currentClawPos);
        }
    }
    public double getClawPosition() {
        return currentClawPos;
    }
    public boolean canToggleSideClaw() {
        return changeSideClaw.milliseconds() > 25;
    }
}
