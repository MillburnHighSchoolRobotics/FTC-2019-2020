package com.millburnrobotics.skystone.autonomous.red;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.millburnrobotics.skystone.Constants;
import com.millburnrobotics.skystone.robot.MohanBot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(group = "auton")
public class RedAutonBaseplateParkBridge extends LinearOpMode {

    private static double BASEPLATE_ALIGNMENT_Y = 49; //24 + 8 + 34/2

    @Override
    public void runOpMode() throws InterruptedException {
        MohanBot robot = new MohanBot(hardwareMap,this,new Pose2d(63, 24 + Constants.DriveConstants.BOT_WIDTH/2, Math.toRadians(270)));
        if (isStopRequested()) return;
        waitForStart();

        robot.strafeTo(new Vector2d(48, BASEPLATE_ALIGNMENT_Y),0.7);

        Thread.sleep(1000);

        robot.strafeTo(new Vector2d(36, BASEPLATE_ALIGNMENT_Y), 0.7);

        robot.hook.hookDown();
        Thread.sleep(1000);

        robot.moveTo(new Vector2d(63, BASEPLATE_ALIGNMENT_Y), 270,0.4,0.6);

        robot.hook.hookUp();
        Thread.sleep(1000);

        robot.strafeTo(new Vector2d(63, 0), 0.7);
        Thread.sleep(100);
        robot.strafeTo(new Vector2d(36, 0), 0.7);
    }
}
