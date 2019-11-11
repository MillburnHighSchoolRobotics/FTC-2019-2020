package org.firstinspires.ftc.teamcode.autonomous.blue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.MohanBot;


@Autonomous(group = "auton")
public class BlueAutonBaseplate extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MohanBot robot = new MohanBot(hardwareMap,this);

        waitForStart();

        if (isStopRequested()) return;

        robot.followTrajectory(
                robot.trajectoryBuilder()
                        .splineTo(new Pose2d(26,28,0), new SplineInterpolator(0,Math.PI))
                        .back(4)
                        .build()
        );

        robot.getHook().hookDown();
        Thread.sleep(1000);
        robot.followTrajectory(
                robot.trajectoryBuilder()
                        .forward(48)
                        .build()
        );
        Thread.sleep(1000);
        robot.getHook().hookUp();
        Thread.sleep(1000);
        robot.followTrajectory(
                robot.trajectoryBuilder()
                        .strafeLeft(69)
                        .build()
        );
    }
}
