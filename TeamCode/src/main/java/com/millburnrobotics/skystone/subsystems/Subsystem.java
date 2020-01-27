package com.millburnrobotics.skystone.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Subsystem {
    public abstract void init(boolean auto);

    public abstract void outputToTelemetry(Telemetry telemetry);

    public abstract void update();
}
