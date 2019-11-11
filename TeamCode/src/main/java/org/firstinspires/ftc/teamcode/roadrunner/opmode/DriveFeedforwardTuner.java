package org.firstinspires.ftc.teamcode.roadrunner.opmode;

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
import org.firstinspires.ftc.teamcode.robot.MohanBot;
import org.firstinspires.ftc.teamcode.util.LoggingUtil;

import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.getMaxRpm;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.rpmToVelocity;

@Config
@Autonomous(group = "drive")
public class DriveFeedforwardTuner extends LinearOpMode {
    public static final double MAX_POWER = 0.7;
    public static final double DISTANCE = 100;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        MohanBot drive = new MohanBot(hardwareMap,this);

        NanoClock clock = NanoClock.system();
        waitForStart();


        double maxVel = rpmToVelocity(getMaxRpm());
        double finalVel = MAX_POWER * maxVel;
        double accel = (finalVel * finalVel) / (2.0 * DISTANCE);
        double rampTime = Math.sqrt(2.0 * DISTANCE / accel);
        double startTime = clock.seconds();
        RampRegression rampRegression = new RampRegression();

        drive.setPose(new Pose2d());
        while (!isStopRequested()) {
            double elapsedTime = clock.seconds() - startTime;
            if (elapsedTime > rampTime) {
                break;
            }
            double vel = accel * elapsedTime;
            double power = vel / maxVel;

            rampRegression.add(elapsedTime, drive.getPose().getX(), power);
            Log.d("DriveFeedForwardVel", elapsedTime + " " + drive.getPose().getX() + " " + power);

            drive.setDrivePower(power);
        }
        drive.setDrivePower(0);

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