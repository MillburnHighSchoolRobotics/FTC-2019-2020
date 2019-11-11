package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.test.roadrunner.mecanum.DriveBase;
import org.firstinspires.ftc.teamcode.test.roadrunner.mecanum.MohanBot2;
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

        DriveBase drive = new MohanBot2(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(24)
                        .build()
        );
        Thread.sleep(1000);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeRight(24)
                        .build()
        );
        Thread.sleep(1000);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .back(24)
                        .build()
        );
        Thread.sleep(1000);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeLeft(24)
                        .build()
        );

    }
}
