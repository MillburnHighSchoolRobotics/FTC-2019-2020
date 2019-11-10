package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.acmerobotics.roadrunner.path.heading.TangentInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.DriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.MohanBot;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleDriveBase;
import org.firstinspires.ftc.teamcode.robot.MillburnRobot;
import org.firstinspires.ftc.teamcode.threads.PositionMonitor;
import org.firstinspires.ftc.teamcode.threads.ThreadManager;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MillburnRobot drive = new MillburnRobot(hardwareMap,this);
        drive.setPose(new Pose2d(0,42,Math.PI));

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(
                drive.trajectoryBuilder()
                        .lineTo(new Vector2d(-39,58))
                        .build()
        );
        drive.followTrajectory(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(-40,32,3*Math.PI/2))
                        .strafeTo(new Vector2d(-42,20))
                        .build()
        );
        drive.followTrajectory(
                drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(0,42,Math.PI))
                        .build()
        );
    }
}
