package com.millburnrobotics.skystone.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.millburnrobotics.skystone.subsystems.MohanBot;

@Autonomous(group = "drive")
public class MoveToTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MohanBot drive = new MohanBot(hardwareMap,this,  new Pose2d());

        waitForStart();

        if (isStopRequested()) return;

//        drive.moveTo(new Vector2d(-24 ,0), 0,0.75,0.25);
//        Thread.sleep(2000);
//        drive.moveTo(new Vector2d(24 ,0), 0,0.75,0.25);
//        Thread.sleep(2000);
//        drive.moveTo(new Vector2d(-24 ,0), 0,0.75,0.25);


        drive.strafeTo(new Vector2d(-12 ,0),0.75);
        Thread.sleep(2000);
        drive.strafeTo(new Vector2d(24 ,0), 0.75);
        Thread.sleep(2000);
        drive.strafeTo(new Vector2d(-12 ,0),0.75);


//        drive.strafeTo(new Vector2d(24,24),0.8);
    }
}
