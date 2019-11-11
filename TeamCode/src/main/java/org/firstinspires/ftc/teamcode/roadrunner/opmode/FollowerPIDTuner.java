package org.firstinspires.ftc.teamcode.roadrunner.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.mecanum.DriveBase;
import org.firstinspires.ftc.teamcode.roadrunner.mecanum.MohanBot2;
import org.firstinspires.ftc.teamcode.threads.PositionMonitor;
import org.firstinspires.ftc.teamcode.threads.ThreadManager;

/*
 * Op mode for tuning follower PID coefficients. The robot drives in a DISTANCE-by-DISTANCE square
 * indefinitely.
 */
@Config
@Autonomous(group = "drive")
public class FollowerPIDTuner extends LinearOpMode {
    public static double DISTANCE = 48;

    @Override
    public void runOpMode() throws InterruptedException {
        ThreadManager manager = ThreadManager.getInstance();
        manager.setHardwareMap(hardwareMap);
        manager.setCurrentAuton(this);
        manager.setupThread("PositionMonitor", PositionMonitor.class);

        DriveBase drive = new MohanBot2(hardwareMap);
//        drive.setPoseEstimate(new Pose2d(-60, 60, 3*Math.PI/2));

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeTo(new Vector2d(DISTANCE,0))
                            .build()
            );
            drive.turnSync(Math.toRadians(180));
            Thread.sleep(1000);
        }
    }
}
