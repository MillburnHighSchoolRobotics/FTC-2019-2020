package com.millburnrobotics.skystone.subsystems;

import com.millburnrobotics.skystone.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static com.millburnrobotics.skystone.Constants.IntakeConstants.INTAKE_IN_POWER;
import static com.millburnrobotics.skystone.Constants.IntakeConstants.INTAKE_OUT_POWER;

public class Intake extends Subsystem {
    public enum IntakeState {
        INTAKE_IN,
        INTAKE_OUT,
        INTAKE_STOP
    }
    private IntakeState state;

    @Override
    public void init(boolean auto) {
        stop();
    }

    @Override
    public void outputToTelemetry(Telemetry telemetry) {
        telemetry.addData("IntakeState", state.name());
    }

    @Override
    public void update() {

    }

    public void intakeIn() {
        setIntakePower(INTAKE_IN_POWER);
        state = IntakeState.INTAKE_IN;
    }
    public void intakeOut() {
        setIntakePower(INTAKE_OUT_POWER);
        state = IntakeState.INTAKE_OUT;
    }
    public void stop() {
        setIntakePower(0);
        state = IntakeState.INTAKE_STOP;
    }
    public void setIntakePower(double power) {
        Robot.getInstance().intakeL.setPower(power);
        Robot.getInstance().intakeR.setPower(power);
    }
    public IntakeState getState() {
        return state;
    }
}
