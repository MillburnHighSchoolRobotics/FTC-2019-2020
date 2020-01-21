package com.millburnrobotics.skystone.test;

import android.util.Log;

import com.millburnrobotics.skystone.Constants;
import com.millburnrobotics.skystone.util.BarkerClass;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.android.OpenCVLoader;


@Autonomous(group = "test")
public class BarkerClassTest extends LinearOpMode {
    static {
        if(OpenCVLoader.initDebug()) {
            Log.d("opencv","yay it works");
        } else {
            Log.d("opencv","nope it doesnt work");
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        BarkerClass barker = new BarkerClass(hardwareMap, Constants.SIDE.RED);

        telemetry.addData("Status", "Loading");
        telemetry.update();

        barker.wake();

        telemetry.addData("Status", "Loaded!");
        telemetry.update();

        waitForStart();

        int pos = barker.bark();

        telemetry.addData("Block",pos);
        telemetry.update();
        Log.d("Block",pos+"");
    }
}
