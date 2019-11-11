package org.firstinspires.ftc.teamcode.test.roadrunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.MohanBot;


@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MohanBot drive = new MohanBot(hardwareMap,this);

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(30,30,0))
                        .build()
        );
        Thread.sleep(1000);
        drive.followTrajectory(
                drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(0,0,0))
                        .build()
        );
    }
}
