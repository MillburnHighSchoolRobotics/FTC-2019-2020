package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.MohanBot;

@Autonomous(group = "drive")
public class MoveToTest extends LinearOpMode {
    public static double DISTANCE = 96;

    @Override
    public void runOpMode() throws InterruptedException {
        MohanBot drive = new MohanBot(hardwareMap,this);

        waitForStart();

        if (isStopRequested()) return;

        drive.moveTo(new Pose2d(24 ,24, 0), 0.8,0.2);
    }
}
