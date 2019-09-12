package org.firstinspires.ftc.teamcode.roadrunner_old.MotionProfiling;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner_old.DriveBase;
import org.firstinspires.ftc.teamcode.roadrunner_old.MohanBot;
import org.firstinspires.ftc.teamcode.threads.PositionMonitor;
import org.firstinspires.ftc.teamcode.threads.ThreadManager;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Autonomous(group = "drive")
public class StraightTest extends LinearOpMode {
    public static double DISTANCE = 12;

    @Override
    public void runOpMode() throws InterruptedException {
        ThreadManager manager = ThreadManager.getInstance();
        manager.setHardwareMap(hardwareMap);
        manager.setupThread("PositionMonitor", PositionMonitor.class);

        DriveBase drive = new MohanBot(hardwareMap);

        Trajectory trajectory = drive.trajectoryBuilder()
                .forward(DISTANCE)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(trajectory);

        Thread.sleep(5000);
    }
}
