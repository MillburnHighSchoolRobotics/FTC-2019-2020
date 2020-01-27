package com.millburnrobotics.skystone.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.millburnrobotics.skystone.subsystems.MohanBot;

@Autonomous(group = "test")
public class StrafeToTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MohanBot drive = new MohanBot(hardwareMap,this,  new Pose2d());

        waitForStart();

        if (isStopRequested()) return;

        for (int i = 0; i < 4; i++) {
            drive.strafeTo(new Vector2d(-24 ,0), 0.3);
            Thread.sleep(1000);
            drive.strafeTo(new Vector2d(0 ,0), 0.3);
            Thread.sleep(1000);
            drive.rotate(90);
            Thread.sleep(1000);
        }
    }
}
