package org.firstinspires.ftc.teamcode.autonomous.blue.park;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.mecanum.DriveBase;
import org.firstinspires.ftc.teamcode.roadrunner.mecanum.MohanBot2;
import org.firstinspires.ftc.teamcode.threads.PositionMonitor;
import org.firstinspires.ftc.teamcode.threads.ThreadManager;


@Autonomous(group = "auton")
public class BlueAutonParkBuildingBridge extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ThreadManager manager = ThreadManager.getInstance();
        manager.setHardwareMap(hardwareMap);
        manager.setCurrentAuton(this);
        manager.setupThread("PositionMonitor", PositionMonitor.class);

        DriveBase drive = new MohanBot2(hardwareMap);
        drive.setPoseEstimate(new Pose2d(24, 63, 0));

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeRight(24)
                        .forward(27)
                        .build()
        );
    }
}
