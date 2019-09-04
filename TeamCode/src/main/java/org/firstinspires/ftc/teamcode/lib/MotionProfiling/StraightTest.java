package org.firstinspires.ftc.teamcode.lib.MotionProfiling;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.DriveBase;
import org.firstinspires.ftc.teamcode.lib.MohanBot;
import org.firstinspires.ftc.teamcode.threads.ThreadManager;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Autonomous(group = "drive")
public class StraightTest extends LinearOpMode {
    public static double DISTANCE = 96;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveBase drive = new MohanBot(hardwareMap);

        Trajectory trajectory = drive.trajectoryBuilder()
                .forward(DISTANCE)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(trajectory);

        telemetry.addData("data", drive.getPoseEstimate().getX());
        telemetry.update();

        Thread.sleep(5000);
    }
}
