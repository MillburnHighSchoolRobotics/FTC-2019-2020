package org.firstinspires.ftc.teamcode.test.trajectory;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.MohanBot;


@Config
@Autonomous(group = "drive")
public class SquareTest extends LinearOpMode {
    public static double DISTANCE = 24;

    @Override
    public void runOpMode() throws InterruptedException {
        MohanBot drive = new MohanBot(hardwareMap,this);

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(drive.trajectoryBuilder().forward(DISTANCE).build());
        drive.followTrajectory(drive.trajectoryBuilder().strafeRight(DISTANCE).build());
        drive.followTrajectory(drive.trajectoryBuilder().back(DISTANCE).build());
        drive.followTrajectory(drive.trajectoryBuilder().strafeLeft(DISTANCE).build());
    }
}
