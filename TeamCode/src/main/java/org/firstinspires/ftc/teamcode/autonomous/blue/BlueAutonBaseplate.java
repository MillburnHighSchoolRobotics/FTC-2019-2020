package org.firstinspires.ftc.teamcode.autonomous.blue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.MohanBot;

import java.util.Vector;


@Autonomous(group = "auton")
public class BlueAutonBaseplate extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MohanBot robot = new MohanBot(hardwareMap,this);

        robot.setPose(new Pose2d(24, 63, Math.toRadians(90)));

        waitForStart();

        if (isStopRequested()) return;

        robot.followTrajectory(
                robot.trajectoryBuilder()
                        .strafeRight(36)
                        .build()
        );

        robot.followTrajectory(
                robot.trajectoryBuilder()
                        .back(48)
                        .build()
        );

        robot.getHook().hookDown();
        Thread.sleep(1000);
        robot.followTrajectory(
                robot.trajectoryBuilder()
                        .forward(60)
                        .build()
        );
        Thread.sleep(1000);
        robot.getHook().hookUp();
        Thread.sleep(1000);
        robot.followTrajectory(
                robot.trajectoryBuilder()
                        .strafeLeft(60)
                        .build()
        );
    }
}
