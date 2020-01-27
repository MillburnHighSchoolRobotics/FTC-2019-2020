package com.millburnrobotics.skystone.test;

import com.millburnrobotics.lib.math.Pose;
import com.millburnrobotics.skystone.subsystems.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(group = "test")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot drive = new Robot(hardwareMap,this, new Pose(0,0,0));

        if (isStopRequested()) return;
        waitForStart();

        drive.follow(0.1,0.9,
                drive.pathGenerator()
                        .splineTo(new Pose(36,36,0))
                        .generatePath()
        );

        Thread.sleep(1000);

        drive.follow(0.1,0.9,
                drive.pathGenerator()
                        .splineTo(new Pose(0,0,0))
                        .generatePath()
        );

        Thread.sleep(1000);
    }
}
