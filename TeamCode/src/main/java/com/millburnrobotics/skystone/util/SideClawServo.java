package com.millburnrobotics.skystone.util;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "util")
public class SideClawServo extends OpMode {
    double barPos, clampPos, inc;
    Servo sideClawBar, sideClawClamp;

    @Override
    public void init() {
        sideClawBar = hardwareMap.servo.get("sideClawBar");
        sideClawClamp = hardwareMap.servo.get("sideClawClamp");
        barPos = clampPos = 0.5;
        inc = 0.1;
    }

    @Override
    public void loop() {
        try {
            Thread.sleep(50);
        } catch (InterruptedException ie) {
            Log.d("IE", ie+"");
        }

        if (gamepad1.dpad_up && clampPos < 1) {
            clampPos += inc;
        }
        if (gamepad1.dpad_down && clampPos > 0) {
            clampPos -= inc;
        }

        if (gamepad1.dpad_left && barPos < 1) {
            barPos += inc;
        }
        if (gamepad1.dpad_right && barPos > 0) {
            barPos -= inc;
        }

        sideClawBar.setPosition(barPos);
        sideClawClamp.setPosition(clampPos);

        telemetry.addData("Bar",barPos+"");
        telemetry.addData("Clamp",clampPos+"");
        telemetry.addData("Inc",inc+"");
        telemetry.update();

        Log.d("Bar",barPos+"");
        Log.d("Clamp",clampPos+"");
        Log.d("Inc",inc+"");
    }

}
