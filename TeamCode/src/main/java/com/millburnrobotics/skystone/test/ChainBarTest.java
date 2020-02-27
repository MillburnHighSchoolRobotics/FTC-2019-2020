package com.millburnrobotics.skystone.test;

import android.util.Log;

import com.millburnrobotics.lib.util.MathUtils;
import com.millburnrobotics.skystone.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "util")
public class ChainBarTest extends OpMode {
    double barPos, inc;
    Servo bar;

    @Override
    public void init() {
        bar = hardwareMap.servo.get(Constants.ChainBarConstants._ChainBarLeft);
        barPos = 0.5;
        inc = 0.05;
    }

    @Override
    public void loop() {
        try {
            Thread.sleep(50);
        } catch (InterruptedException ie) {
            Log.d("IE", ie+"");
        }


        if ((gamepad1.dpad_left || gamepad1.right_bumper) && barPos < 1) {
            barPos += inc;
        }
        if ((gamepad1.dpad_right || MathUtils.equals(gamepad1.right_trigger, 1, 0.05)) && barPos > 0) {
            barPos -= inc;
        }

        bar.setPosition(barPos);

        telemetry.addData("Bar",barPos+"");
        telemetry.addData("Inc",inc+"");
        telemetry.update();

        Log.d("Bar",barPos+"");
        Log.d("Inc",inc+"");
    }

}
