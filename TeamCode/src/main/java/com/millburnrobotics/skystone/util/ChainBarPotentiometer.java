package com.millburnrobotics.skystone.util;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;

@Autonomous(group = "util")
public class ChainBarPotentiometer extends LinearOpMode {
    private AnalogInput chainBarPot;

    @Override
    public void runOpMode() throws InterruptedException {
        chainBarPot = hardwareMap.get(AnalogInput.class, "chainBarPot");

        waitForStart();

        if (isStopRequested()) return;

        while (true) {
            telemetry.addData("Current voltage", chainBarPot.getVoltage());
            telemetry.update();
            Log.d("Voltage", chainBarPot.getVoltage()+"");
        }
    }

}
