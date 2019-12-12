package org.firstinspires.ftc.teamcode.test;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.robot.GlobalConstants;
import org.firstinspires.ftc.teamcode.util.BarkerClass;
import org.firstinspires.ftc.teamcode.util.SkystoneDetector;
import org.firstinspires.ftc.teamcode.util.VuforiaLocalizerImplSubclass;
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
        BarkerClass barker = new BarkerClass(hardwareMap, GlobalConstants.SIDE.RED);

        telemetry.addData("Status", "Loading");
        telemetry.update();

        barker.init();

        telemetry.addData("Status", "Loaded!");
        telemetry.update();

        waitForStart();

        int pos = barker.bark();

        telemetry.addData("Block",pos);
        telemetry.update();
        Log.d("Block",pos+"");
    }
}
