package org.firstinspires.ftc.teamcode.util;

import android.app.Activity;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

@Autonomous(group = "util")
public class ChainBarPot extends LinearOpMode {
    AnalogInput pot;
    DcMotor chainBar;

    private double currentVoltage;

    @Override
    public void runOpMode() {
        pot = hardwareMap.get(AnalogInput.class, "chainBarPot");
        chainBar = hardwareMap.get(DcMotor.class, "chainBar");

        waitForStart();

        if(isStopRequested()) return;

        while (!shouldStop()) {
            currentVoltage = pot.getVoltage();
            telemetry.addData("Voltage", (double)Math.round(currentVoltage * 1000) / 1000);
            telemetry.update();
        }
    }
    public static boolean shouldStop() {
        Activity currActivity = AppUtil.getInstance().getActivity();
        OpModeManagerImpl manager = OpModeManagerImpl.getOpModeManagerOfActivity(currActivity);
        OpMode currentOpMode = manager.getActiveOpMode();
        return currentOpMode instanceof LinearOpMode &&
                ((LinearOpMode) currentOpMode).isStarted() &&
                ((LinearOpMode) currentOpMode).isStopRequested();
    }
}
