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
import org.firstinspires.ftc.teamcode.threads.PositionMonitor;
import org.firstinspires.ftc.teamcode.threads.ThreadManager;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ThreadManager manager = ThreadManager.getInstance();
        manager.setHardwareMap(hardwareMap);
        manager.setCurrentAuton(this);
        manager.setupThread("PositionMonitor", PositionMonitor.class);

        DriveBase drive = new MohanBot(hardwareMap);

        drive.setPoseEstimate(new Pose2d(-48, 60, 3*Math.PI/2));
//        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(0,0,0), new SplineInterpolator(3*Math.PI/2, Math.PI))
//                        .reverse()
//                        .splineTo(new Pose2d(-48, 60,3*Math.PI/2)/*, new SplineInterpolator(Math.PI/2, Math.PI)*/)
//                        .reverse()
//                        .strafeTo(new Vector2d(-24, 0))
//                        .strafeTo(new Vector2d(-48, 60))
//                        .splineTo(new Pose2d(-48,0, 3*Math.PI/2), new ConstantInterpolator(3*Math.PI/2))
//
                        .build()
        );
//        drive.followTrajectorySync(
//                drive.trajectoryBuilder()
//                        .splineTo(new Pose2d(30,30,0))
//                        .build()
//        );
//        Thread.sleep(2000);
//        drive.followTrajectorySync(
//                drive.trajectoryBuilder()
//                        .reverse()
//                        .splineTo(new Pose2d(0,0,0))
//                        .build()
//        );
    }
}
