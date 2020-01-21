package com.millburnrobotics.skystone.autonomous.blue;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.millburnrobotics.skystone.robot.GlobalConstants;
import com.millburnrobotics.skystone.robot.MohanBot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.android.OpenCVLoader;


@Autonomous(group = "auton")
public class BlueAutonNew extends LinearOpMode {
    static {
        if (OpenCVLoader.initDebug()) {
            Log.d("opencv", "yay it works");
        } else {
            Log.d("opencv", "nope it doesnt work");
        }
    }

    MohanBot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new MohanBot(hardwareMap, this, (new Pose2d(new Vector2d(-72+ GlobalConstants.BOT_WIDTH/2.0,-48+ GlobalConstants.BOT_LENGTH/2.0), 0)));

        waitForStart();

        if (isStopRequested()) return;

        int pos = 1;
        telemetry.addData("block",pos);
        telemetry.update();

        if (pos == 1) {
            sample12(GlobalConstants.BLUE_BLOCK_4);
            Thread.sleep(1000);
            deliver(GlobalConstants.BLUE_DELIVERY_1);
            Thread.sleep(1000);

            cycle(GlobalConstants.BLUE_BLOCK_1);
            Thread.sleep(1000);
            deliver(GlobalConstants.BLUE_DELIVERY_2);
            Thread.sleep(1000);

            cycle(GlobalConstants.BLUE_BLOCK_2);
            Thread.sleep(1000);
            deliver(GlobalConstants.BLUE_DELIVERY_3);
            Thread.sleep(1000);

            cycle(GlobalConstants.BLUE_BLOCK_3);
            Thread.sleep(1000);
            deliver(GlobalConstants.BLUE_DELIVERY_4);
            Thread.sleep(1000);
        } else if (pos == 2) {

        }
//        park();
    }
    public void sample12(Vector2d block) {
        robot.follow(0.1,0.7,
                robot.path(Math.toRadians(270))
                        .lineTo(new Vector2d(-72+ GlobalConstants.BOT_WIDTH/2.0+4,-48+ GlobalConstants.BOT_LENGTH/2.0).rotated(-Math.PI/2))
                        .splineTo(new Pose2d(block.rotated(-Math.PI/2),Math.toRadians(270)))
                        .build(),
                new double[]{0,0}, true, 6
        );
    }
    public void cycle(Vector2d block) {
        robot.follow(0.2,0.85,
                robot.path(Math.toRadians(170))
                        .splineTo(new Pose2d(new Vector2d(-38,0).rotated(-Math.PI/2),Math.toRadians(180)))
                        .splineTo(new Pose2d(block.rotated(-Math.PI/2),Math.toRadians(190)))
                        .build(),
                new double[]{0,0}, true, 6
        );
    }
    public void deliver(Vector2d deliver) {
        robot.follow(0.2,0.85,
                robot.path(Math.toRadians(10))
                        .splineTo(new Pose2d(new Vector2d(-38,0).rotated(-Math.PI/2),Math.toRadians(0)))
                        .splineTo(new Pose2d(deliver.rotated(-Math.PI/2),Math.toRadians(345)))
                        .build(),
                new double[]{0,0}, true, 8
        );
    }
    public void park() {
        robot.follow(0.2,1,
                robot.path(Math.toRadians(0))
                        .lineTo(GlobalConstants.BLUE_FOUNDATION.rotated(-Math.PI/2))
                        .build(),
                new double[]{90}, true, 10
        );
        robot.follow(0.3,1,
                robot.path(Math.toRadians(90))
                        .back(10)
                        .build(),
                new double[]{90}
        );
        robot.follow(0.2,1,
                robot.path(Math.toRadians(0))
                        .lineTo(new Vector2d(-72+ GlobalConstants.BOT_WIDTH/2.0+4, GlobalConstants.BLUE_FOUNDATION.getY()).rotated(-Math.PI/2))
                        .build(),
                new double[]{180}
        );
        robot.follow(0.5,1,
                robot.path(Math.toRadians(180))
                        .splineTo(new Pose2d(GlobalConstants.BLUE_BRIDGE_PARK.rotated(-Math.PI/2),Math.toRadians(180)))
                        .build(),
                new double[]{90}, false
        );
    }
}
