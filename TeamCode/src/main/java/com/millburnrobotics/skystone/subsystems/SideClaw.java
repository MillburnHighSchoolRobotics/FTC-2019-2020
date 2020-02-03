package com.millburnrobotics.skystone.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
        setArmPosition(0);
    }
    public void armDown() {
        setArmPosition(1);
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
