package org.firstinspires.ftc.teamcode.test;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.MohanBot;
import org.firstinspires.ftc.teamcode.util.FUCKMYLIFEClass;
import org.opencv.android.OpenCVLoader;


@Autonomous(group = "auton")
public class FUCKMETest extends LinearOpMode {
    static {
        if(OpenCVLoader.initDebug()) {
            Log.d("opencv","yay it works");
        } else {
            Log.d("opencv","nope it doesnt work");
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
//        MohanBot mohanBot = new MohanBot(hardwareMap,this);
//
//        mohanBot.getHook().hookUp();
//        mohanBot.getChainBar().openClaw();

        FUCKMYLIFEClass fuck = new FUCKMYLIFEClass(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

//        drive.followTrajectory(
//                drive.trajectoryBuilder()
//                        .forward(10)
//                        .build()
//        );

        int pos = fuck.getPos();
        telemetry.addData("fuck",pos);
        telemetry.update();
        Log.d("fuck",pos+"");
//        while (!isStopRequested()) {
//            Thread.sleep(10);
//        }
    }
}
