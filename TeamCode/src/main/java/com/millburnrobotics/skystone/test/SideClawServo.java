package com.millburnrobotics.skystone.test;

import android.util.Log;

import com.millburnrobotics.lib.util.MathUtils;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "util")
public class SideClawServo extends OpMode {
    double barPos, clampPos, inc;
    Servo sideClawBar, sideClawClamp;

    @Override
    public void init() {
        sideClawBar = hardwareMap.servo.get("sideClawArmR");
        sideClawClamp = hardwareMap.servo.get("sideClawClawR");
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

        if ((gamepad1.dpad_up || gamepad1.left_bumper) && clampPos < 1) {
            clampPos += inc;
        }
        if ((gamepad1.dpad_down || MathUtils.equals(gamepad1.left_trigger, 1, 0.05)) && clampPos > 0) {
            clampPos -= inc;
        }

        if ((gamepad1.dpad_left || gamepad1.right_bumper) && barPos < 1) {
            barPos += inc;
        }
        if ((gamepad1.dpad_right || MathUtils.equals(gamepad1.right_trigger, 1, 0.05)) && barPos > 0) {
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
