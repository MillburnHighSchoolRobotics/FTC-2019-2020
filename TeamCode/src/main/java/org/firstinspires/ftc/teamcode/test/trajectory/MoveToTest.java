package org.firstinspires.ftc.teamcode.test.trajectory;

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
        MohanBot drive = new MohanBot(hardwareMap,this, new Pose2d(-63,-36,Math.toRadians(270)));

        waitForStart();

        if (isStopRequested()) return;

        drive.moveTo(new Pose2d(0 ,-24, 270), .4);
    }
}
