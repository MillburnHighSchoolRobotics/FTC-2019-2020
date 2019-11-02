package org.firstinspires.ftc.teamcode.drive.opmode;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.tuning.AccelRegression;
import com.acmerobotics.roadrunner.tuning.RampRegression;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.mecanum.DriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.MohanBot;
import org.firstinspires.ftc.teamcode.threads.PositionMonitor;
import org.firstinspires.ftc.teamcode.threads.ThreadManager;
import org.firstinspires.ftc.teamcode.util.LoggingUtil;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.getMaxRpm;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.rpmToVelocity;

@Config
@Autonomous(group = "drive")
public class DriveFeedforwardTunerAcceleration extends LinearOpMode {
    public static final double MAX_POWER = 0.7;
    public static final double DISTANCE = 100;

    @Override
    public void runOpMode() throws InterruptedException {
        ThreadManager manager = ThreadManager.getInstance();
        manager.setHardwareMap(hardwareMap);
        manager.setCurrentAuton(this);
        manager.setupThread("PositionMonitor", PositionMonitor.class);

        DriveBase drive = new MohanBot(hardwareMap);

        NanoClock clock = NanoClock.system();

        waitForStart();

        double maxVel = rpmToVelocity(getMaxRpm());
        double maxPowerTime = DISTANCE / maxVel;


        AccelRegression accelRegression = new AccelRegression();

        drive.setPoseEstimate(new Pose2d());
        drive.setDrivePower(new Pose2d(MAX_POWER, 0.0, 0.0));
        Thread.sleep(250);
        double startTime = clock.seconds();
        while (!isStopRequested()) {
            double elapsedTime = clock.seconds() - startTime;
            if (elapsedTime > maxPowerTime) {
                break;
            }

            accelRegression.add(elapsedTime, drive.getPoseEstimate().getX(), MAX_POWER);

//            Log.d("acceltuner", "time: " + String.valueOf(elapsedTime));
//            Log.d("acceltuner", "pos: " + String.valueOf(drive.getPoseEstimate().getX()));
//            Log.d("acceltuner", "powertime: " + String.valueOf(maxPowerTime));
//            Log.d("acceltuner", "maxrpm: " + getMaxRpm());
//            Log.d("acceltuner", "maxvel: " + rpmToVelocity(getMaxRpm()));

            Log.d("literallywhatever?!", drive.getPoseEstimate().getX() + "\t" + elapsedTime);

            drive.updatePoseEstimate();
        }
        drive.setDrivePower(new Pose2d(0.0, 0.0, 0.0));


        AccelRegression.AccelResult accelResult = accelRegression.fit(kV, kStatic);

        accelRegression.save(LoggingUtil.getLogFile(Misc.formatInvariant(
                "DriveAccelRegression-%d.csv", System.currentTimeMillis())));


        telemetry.log().clear();
        telemetry.log().add("Constant power test complete");
        telemetry.log().add(Misc.formatInvariant("kA = %.9f (R^2 = %.9f)",
                accelResult.kA, accelResult.rSquare));
            telemetry.update();
        Log.d("literallywhatever?!", accelResult.kA + "\t" + accelResult.rSquare);

        while (!isStopRequested()) {
            idle();
        }
    }
}