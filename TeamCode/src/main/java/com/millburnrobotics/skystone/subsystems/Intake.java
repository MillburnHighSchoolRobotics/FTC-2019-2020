package com.millburnrobotics.skystone.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static com.millburnrobotics.skystone.Constants.IntakeConstants.INTAKE_IN_POWER;
import static com.millburnrobotics.skystone.Constants.IntakeConstants.INTAKE_OUT_POWER;

public class Intake extends Subsystem {
    @Override
    public void init(boolean auto) {
        stop();
    }

    @Override
    public void outputToTelemetry(Telemetry telemetry) {

    }

    @Override
    public void update() {

    }

    public void intakeIn() {
        setIntakePower(INTAKE_IN_POWER);
    }
    public void intakeOut() {
        setIntakePower(INTAKE_OUT_POWER);
    }
    public void stop() {
        setIntakePower(0);
    }
    public void setIntakePower(double power) {
        Robot.getInstance().intakeL.setPower(power);
        Robot.getInstance().intakeR.setPower(power);
    }
}
