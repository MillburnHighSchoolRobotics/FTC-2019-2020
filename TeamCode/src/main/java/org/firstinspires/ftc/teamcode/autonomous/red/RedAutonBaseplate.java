package org.firstinspires.ftc.teamcode.autonomous.red;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.MohanBot;


@Autonomous(group = "auton")
public class RedAutonBaseplate extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MohanBot robot = new MohanBot(hardwareMap,this,new Pose2d(-24, 63, Math.toRadians(270)));

        waitForStart();

        if (isStopRequested()) return;

        robot.followTrajectory(
                robot.trajectoryBuilder()
                        .strafeLeft(36)
                        .build()
        );

        robot.followTrajectory(
                robot.trajectoryBuilder()
                        .back(48)
                        .build()
        );

        robot.hook.hookDown();
        Thread.sleep(1000);
        robot.followTrajectory(
                robot.trajectoryBuilder()
                        .forward(60)
                        .build()
        );
        Thread.sleep(1000);
        robot.hook.hookUp();
        Thread.sleep(1000);
        robot.followTrajectory(
                robot.trajectoryBuilder()
                        .strafeRight(60)
                        .build()
        );
    }
}
