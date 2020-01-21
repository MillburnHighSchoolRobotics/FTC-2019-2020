package com.millburnrobotics.skystone.test;

import android.util.Log;

import com.millburnrobotics.skystone.util.SkystoneDetector;
import com.millburnrobotics.skystone.util.VuforiaLocalizerImplSubclass;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import com.millburnrobotics.skystone.robot.GlobalConstants;

import org.opencv.android.OpenCVLoader;

@Disabled
@Deprecated
@Autonomous(group = "test")
public class SkystoneDetectorTest extends LinearOpMode {
    static {
        if(OpenCVLoader.initDebug()) {
            Log.d("opencv","yay it works");
        } else {
            Log.d("opencv","nope it doesnt work");
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        GlobalConstants.side = GlobalConstants.SIDE.RED;

        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(hardwareMap.appContext.getResources().getIdentifier("Webcam1", "id", hardwareMap.appContext.getPackageName()));

        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        params.vuforiaLicenseKey = GlobalConstants.VUFORIA_KEY;
        telemetry.addData("Vuforia Status: ", "Loading...");
        telemetry.update();
        VuforiaLocalizerImplSubclass vuforiaInstance = new VuforiaLocalizerImplSubclass(params);

        telemetry.addData("Vuforia Status: ", "Loaded!");
        telemetry.update();

        waitForStart();

        SkystoneDetector fuck = new SkystoneDetector(vuforiaInstance);

        int pos = fuck.getPos();
        telemetry.addData("fuck",pos);
        telemetry.update();
        Log.d("fuck",pos+"");
    }
}
