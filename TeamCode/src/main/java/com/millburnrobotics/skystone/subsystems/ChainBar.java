package com.millburnrobotics.skystone.subsystems;

import com.millburnrobotics.skystone.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static com.millburnrobotics.skystone.Constants.ChainBarConstants.CAPSTONE_CLOSE;
import static com.millburnrobotics.skystone.Constants.ChainBarConstants.CAPSTONE_OPEN;
import static com.millburnrobotics.skystone.Constants.ChainBarConstants.CHAINBARL_ALL_OUT_POS;
import static com.millburnrobotics.skystone.Constants.ChainBarConstants.CHAINBARL_IN_POS;
import static com.millburnrobotics.skystone.Constants.ChainBarConstants.CHAINBARL_OUT_POS;
import static com.millburnrobotics.skystone.Constants.ChainBarConstants.CHAINBARL_UP_POS;
import static com.millburnrobotics.skystone.Constants.ChainBarConstants.CHAINBARR_ALL_OUT_POS;
import static com.millburnrobotics.skystone.Constants.ChainBarConstants.CHAINBARR_IN_POS;
import static com.millburnrobotics.skystone.Constants.ChainBarConstants.CHAINBARR_OUT_POS;
import static com.millburnrobotics.skystone.Constants.ChainBarConstants.CHAINBARR_UP_POS;
import static com.millburnrobotics.skystone.Constants.ChainBarConstants.CHAINBAR_CLAW_CLOSE;
import static com.millburnrobotics.skystone.Constants.ChainBarConstants.CHAINBAR_CLAW_OPEN;
import static com.millburnrobotics.skystone.Constants.ChainBarConstants.CHAINBAR_CLAW_OPENISH;

public class ChainBar extends Subsystem {
    public enum CapstoneState {
        CAPSTONE_CLOSE,
        CAPSTONE_OPEN
    }
    private CapstoneState state;
    double currentLeftPos;
    double currentRightPos;
    ElapsedTime changeChainbar = new ElapsedTime();
    ElapsedTime changeClaw = new ElapsedTime();
    ElapsedTime changeCapstone = new ElapsedTime();

    boolean chainBarIn = false;
    ElapsedTime chainBarInTimer = new ElapsedTime();
    @Override
    public void init(boolean auto) {
        if (!auto) {
            changeChainbar.reset();
            chainBarUp();
            clawOpen();
        }
        capstoneClose();
    }

    @Override
    public void outputToTelemetry(Telemetry telemetry) {
        telemetry.addData("ChainBarPositionR", currentLeftPos);
        telemetry.addData("ChainBarPositionL", currentRightPos);
    }

    @Override
    public void update() {

    }
    public void chainBarUp() {
        setChainBarPosition(CHAINBARL_UP_POS, CHAINBARR_UP_POS);
    }
    public void chainBarIn() {
        setChainBarPosition(CHAINBARL_IN_POS, CHAINBARR_IN_POS);
    }
    public void chainBarOut() {
        setChainBarPosition(CHAINBARL_OUT_POS, CHAINBARR_OUT_POS);
    }
    public void chainBarOutAll() {
        setChainBarPosition(CHAINBARL_ALL_OUT_POS, CHAINBARR_ALL_OUT_POS);
    }
    public void chainBarInAuto() {
        chainBarIn();
        chainBarIn = true;
        chainBarInTimer.reset();
    }
    public void chainBarInUpdate() {
        if (chainBarInTimer.milliseconds() > 1000) {
            chainBarIn = false;
            chainBarUp();
        }
    }
    public boolean isChainBarIn() {
        return chainBarIn;
    }
    public void setChainBarPosition(double leftPos, double rightPos) {
        this.currentLeftPos = leftPos;
        this.currentRightPos = rightPos;
        changeChainbar.reset();
        Robot.getInstance().chainBarL.setPosition(currentLeftPos);
        Robot.getInstance().chainBarR.setPosition(currentRightPos);
    }
    public void clawClose() {
        setClawPosition(CHAINBAR_CLAW_CLOSE);
    }
    public void clawOpen() {
        setClawPosition(CHAINBAR_CLAW_OPEN);
    }
    public void clawOpenish() {
        setClawPosition(CHAINBAR_CLAW_OPENISH);
    }
    public void setClawPosition(double pos) {
        changeClaw.reset();
        Robot.getInstance().claw.setPosition(pos);
    }
    public double getChainBarLPosition() {
        return currentLeftPos;
    }
    public double getChainBarRPosition() {
        return currentRightPos;
    }
    public boolean canToggleChainBar() {
        return changeChainbar.milliseconds() > 25;
    }
    public boolean canToggleClaw() {
        return changeClaw.milliseconds() > 50;
    }

    public void capstoneClose() {
        state = CapstoneState.CAPSTONE_CLOSE;
        setCapstonePosition(CAPSTONE_CLOSE);
    }
    public void capstoneOpen() {
        state = CapstoneState.CAPSTONE_OPEN;
        setCapstonePosition(CAPSTONE_OPEN);
    }
    public void setCapstonePosition(double pos) {
        Robot.getInstance().capstone.setPosition(pos);
        changeCapstone.reset();
    }
    public boolean canToggleCapstone() {
        return (changeCapstone.milliseconds() > 250);
    }
    public CapstoneState getState() {
        return state;
    }
}
