package com.millburnrobotics.skystone.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Subsystem {
    public String TAG;
    public abstract void init(boolean auto);

    public abstract void outputToTelemetry(Telemetry telemetry, TelemetryPacket packet);

    public abstract void update();
}
