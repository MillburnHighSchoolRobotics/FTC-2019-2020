package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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
public class MindstormSquare extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ThreadManager manager = ThreadManager.getInstance();
        manager.setHardwareMap(hardwareMap);
        manager.setCurrentAuton(this);
        manager.setupThread("PositionMonitor", PositionMonitor.class);

        DriveBase drive = new MohanBot(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(24)
                        .build()
        );
//        Thread.sleep(1000);
//        drive.followTrajectorySync(
//                drive.trajectoryBuilder()
//                        .strafeRight(24)
//                        .build()
//        );
        Thread.sleep(1000);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()
                        .forward(24)
                        .build()
        );
//        Thread.sleep(1000);
//        drive.followTrajectorySync(
//                drive.trajectoryBuilder()
//                        .strafeLeft(24)
//                        .build()
//        );

    }
}
