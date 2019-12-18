package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.MohanBot;


@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MohanBot drive = new MohanBot(hardwareMap,this, new Pose2d(0,0,0));
        if (isStopRequested()) return;
        waitForStart();

        drive.follow(0.1,0.9,
                drive.path(0)
                        .splineTo(new Pose2d(new Vector2d(36,36).rotated(-Math.PI/2),0))
                        .build()
        );

        Thread.sleep(1000);

        drive.follow(0.1,0.9,
                drive.path(Math.PI/2)
                        .splineTo(new Pose2d(new Vector2d(0,0).rotated(-Math.PI/2),Math.PI))
                        .build()
        );
        Thread.sleep(1000);
    }
}
