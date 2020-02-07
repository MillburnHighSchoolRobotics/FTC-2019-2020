package com.millburnrobotics.skystone.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static com.millburnrobotics.skystone.Constants.SideClawConstants.SIDE_ARM_DOWN_POS;
import static com.millburnrobotics.skystone.Constants.SideClawConstants.SIDE_ARM_MID_POS;
import static com.millburnrobotics.skystone.Constants.SideClawConstants.SIDE_ARM_UP_POS;

public class SideClaw extends Subsystem {
    @Override
    public void init(boolean auto) {
    }

    @Override
    public void outputToTelemetry(Telemetry telemetry) {

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

    public void setArmPosition(double pos) {
        Robot.getInstance().sideClawArm.setPosition(pos);
    }

    public void clawUp() {
        setClawPosition(0);
    }
    public void clawDown() {
        setClawPosition(1);
    }
    public void setClawPosition(double pos) {
        Robot.getInstance().sideClawClaw.setPosition(pos);
    }
}
