package org.firstinspires.ftc.teamcode.test.trajectory;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.MohanBot;

import static org.firstinspires.ftc.teamcode.robot.MohanBot.convertPose;


@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MohanBot drive = new MohanBot(hardwareMap,this, new Pose2d(0,0,0));
        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(
                drive.trajectoryBuilder()
                        .splineTo(convertPose(new Pose2d(24,24,0)), new ConstantInterpolator(0))
                        .build()
        );
        Thread.sleep(1000);
    }
}
