package org.firstinspires.ftc.teamcode.drive.opmode;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.tuning.RampRegression;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.drive.mecanum.DriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.MohanBot;
import org.firstinspires.ftc.teamcode.threads.PositionMonitor;
import org.firstinspires.ftc.teamcode.threads.ThreadManager;
import org.firstinspires.ftc.teamcode.util.LoggingUtil;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.getMaxRpm;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.rpmToVelocity;

/*
 * Op mode for computing kV, kStatic, and kA from various drive routines. For the curious, here's an
 * outline of the procedure:
 *   1. Slowly ramp the motor power and record encoder values along the way.
 *   2. Run a linear regression on the encoder velocity vs. motor power plot to obtain a slope (kV)
 *      and an optional intercept (kStatic).
 *   3. Accelerate the robot (apply constant power) and record the encoder counts.
 *   4. Adjust the encoder data based on the velocity tuning data and find kA with another linear
 *      regression.
 */
@Config
@Autonomous(group = "drive")
public class DriveFeedforwardTuner extends LinearOpMode {
    public static final double MAX_POWER = 0.7;
    public static final double DISTANCE = 100;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ThreadManager manager = ThreadManager.getInstance();
        manager.setHardwareMap(hardwareMap);
        manager.setCurrentAuton(this);
        manager.setupThread("PositionMonitor", PositionMonitor.class);
        DriveBase drive = new MohanBot(hardwareMap);

        NanoClock clock = NanoClock.system();
        waitForStart();


        double maxVel = rpmToVelocity(getMaxRpm());
        double finalVel = MAX_POWER * maxVel;
        double accel = (finalVel * finalVel) / (2.0 * DISTANCE);
        double rampTime = Math.sqrt(2.0 * DISTANCE / accel);
        double startTime = clock.seconds();
        RampRegression rampRegression = new RampRegression();

        drive.setPoseEstimate(new Pose2d());
        while (!isStopRequested()) {
            double elapsedTime = clock.seconds() - startTime;
            if (elapsedTime > rampTime) {
                break;
            }
            double vel = accel * elapsedTime;
            double power = vel / maxVel;

            rampRegression.add(elapsedTime, drive.getPoseEstimate().getX(), power);
            Log.d("DriveFeedForwardVel", elapsedTime + " " + drive.getPoseEstimate().getX() + " " + power);

            drive.setDrivePower(new Pose2d(power, 0.0, 0.0));
            drive.updatePoseEstimate();
        }
        drive.setDrivePower(new Pose2d(0.0, 0.0, 0.0));

        RampRegression.RampResult rampResult = rampRegression.fit(true);

        rampRegression.save(LoggingUtil.getLogFile(Misc.formatInvariant(
                "DriveRampRegression-%d.csv", System.currentTimeMillis())));

        telemetry.clearAll();
        telemetry.log().add(Misc.formatInvariant("kV = %.5f, kStatic = %.5f (R^2 = %.2f)",
                    rampResult.kV, rampResult.kStatic, rampResult.rSquare));

        Log.d("DriveFeedForwardVel", rampResult.kV + "\t" + rampResult.kStatic + "\t" + rampResult.rSquare);

        while (!isStopRequested()) {
            idle();
        }
    }
}