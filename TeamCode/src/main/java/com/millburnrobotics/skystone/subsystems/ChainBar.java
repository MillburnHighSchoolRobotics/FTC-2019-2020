package com.millburnrobotics.skystone.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static com.millburnrobotics.skystone.Constants.ChainBarConstants.CHAINBARL_IN_POS;
import static com.millburnrobotics.skystone.Constants.ChainBarConstants.CHAINBARL_OUT_POS;
import static com.millburnrobotics.skystone.Constants.ChainBarConstants.CHAINBARL_UP_POS;
import static com.millburnrobotics.skystone.Constants.ChainBarConstants.CHAINBARR_IN_POS;
import static com.millburnrobotics.skystone.Constants.ChainBarConstants.CHAINBARR_OUT_POS;
import static com.millburnrobotics.skystone.Constants.ChainBarConstants.CHAINBARR_UP_POS;

public class ChainBar extends Subsystem {
    public enum ChainBarState {
        CHAIN_BAR_IN,
        CHAIN_BAR_OUT,
        CHAIN_BAR_UP
    }
    private ChainBarState state;

    @Override
    public void init(boolean auto) {
//        chainBarIn();
    }

    @Override
    public void outputToTelemetry(Telemetry telemetry) {
//        telemetry.addData("ChainBarState", state.name());
    }

    @Override
    public void update() {

    }
    public void chainBarUp() {
        setChainBarPosition(CHAINBARL_UP_POS, CHAINBARR_UP_POS);
        state = ChainBarState.CHAIN_BAR_UP;
    }
    public void chainBarIn() {
        setChainBarPosition(CHAINBARL_IN_POS, CHAINBARR_IN_POS);
        state = ChainBarState.CHAIN_BAR_IN;
    }
    public void chainBarOut() {
        setChainBarPosition(CHAINBARL_OUT_POS, CHAINBARR_OUT_POS);
        state = ChainBarState.CHAIN_BAR_OUT;
    }
    public void setChainBarPosition(double leftPos, double rightPos) {
        Robot.getInstance().chainBarL.setPosition(leftPos);
        Robot.getInstance().chainBarR.setPosition(rightPos);
    }
    public ChainBarState getState() {
        return state;
    }
}
