package org.firstinspires.ftc.teamcode.roadrunner.opmode;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.tuning.AccelRegression;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.robot.MohanBot;
import org.firstinspires.ftc.teamcode.util.LoggingUtil;

import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.getMaxRpm;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.kV;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.rpmToVelocity;

@Config
@Autonomous(group = "drive")
public class DriveFeedforwardTunerAcceleration extends LinearOpMode {
    public static final double MAX_POWER = 0.7;
    public static final double DISTANCE = 100;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        MohanBot drive = new MohanBot(hardwareMap,this);
        NanoClock clock = NanoClock.system();

        waitForStart();

        double maxVel = rpmToVelocity(getMaxRpm());
        double maxPowerTime = DISTANCE / maxVel;
        AccelRegression accelRegression = new AccelRegression();
        double startTime = clock.seconds();


        drive.setPose(new Pose2d(0,0,0));
        drive.setDrivePower(MAX_POWER);
        while (!isStopRequested()) {
            double elapsedTime = clock.seconds() - startTime;
            if (elapsedTime > maxPowerTime) {
                break;
            }

            accelRegression.add(elapsedTime, drive.getPose().getX(), MAX_POWER);
            Log.d("DriveFeedForwardAcc", drive.getPose().getX() + "\t" + elapsedTime);

        }
        drive.setDrivePower(0);


        AccelRegression.AccelResult accelResult = accelRegression.fit(kV, kStatic);

        accelRegression.save(LoggingUtil.getLogFile(Misc.formatInvariant(
                "DriveAccelRegression-%d.csv", System.currentTimeMillis())));


        telemetry.log().clear();
        telemetry.log().add("Constant power test complete");
        telemetry.log().add(Misc.formatInvariant("kA = %.9f (R^2 = %.9f)",
                accelResult.kA, accelResult.rSquare));
            telemetry.update();
        Log.d("DriveFeedForwardAcc", accelResult.kA + "\t" + accelResult.rSquare);

        while (!isStopRequested()) {
            idle();
        }
    }
}