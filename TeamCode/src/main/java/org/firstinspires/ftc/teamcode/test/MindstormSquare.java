package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.MohanBot;
import org.firstinspires.ftc.teamcode.threads.PositionMonitor;
import org.firstinspires.ftc.teamcode.threads.ThreadManager;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "test")
public class MindstormSquare extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ThreadManager manager = ThreadManager.getInstance();
        manager.setHardwareMap(hardwareMap);
        manager.setCurrentAuton(this);
        manager.setupThread("PositionMonitor", PositionMonitor.class);

        MohanBot drive = new MohanBot(hardwareMap,this);

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(
                drive.trajectoryBuilder()
                        .forward(24)
                        .build()
        );
        Thread.sleep(1000);
        drive.followTrajectory(
                drive.trajectoryBuilder()
                        .strafeRight(24)
                        .build()
        );
        Thread.sleep(1000);
        drive.followTrajectory(
                drive.trajectoryBuilder()
                        .back(24)
                        .build()
        );
        Thread.sleep(1000);
        drive.followTrajectory(
                drive.trajectoryBuilder()
                        .strafeLeft(24)
                        .build()
        );

    }
}
