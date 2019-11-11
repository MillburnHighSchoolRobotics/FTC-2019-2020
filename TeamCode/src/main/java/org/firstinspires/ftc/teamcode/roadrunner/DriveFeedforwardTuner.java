package org.firstinspires.ftc.teamcode.roadrunner;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.tuning.AccelRegression;
import com.acmerobotics.roadrunner.tuning.RampRegression;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.robot.MohanBot;
import org.firstinspires.ftc.teamcode.util.LoggingUtil;

import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.getMaxRpm;
import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.rpmToVelocity;


@Autonomous(group = "drive")
public class DriveFeedforwardTuner extends LinearOpMode {
    public static final double MAX_POWER = 0.7;
    public static final double DISTANCE = 100;

    @Override
    public void runOpMode() throws InterruptedException {
        MohanBot robot = new MohanBot(hardwareMap,this);
        NanoClock clock = NanoClock.system();

        waitForStart();

        double maxVel = rpmToVelocity(getMaxRpm());
        double finalVel = MAX_POWER*maxVel;
        double accel = (finalVel*finalVel)/(2.0*DISTANCE);
        double rampTime = Math.sqrt(2.0*DISTANCE/accel);

        double startTime = clock.seconds();
        RampRegression rampRegression = new RampRegression();

        while (!isStopRequested()) {
            double elapsedTime = clock.seconds() - startTime;
            if (elapsedTime > rampTime) {
                break;
            }
            double vel = accel*elapsedTime;
            double power = vel/maxVel;

            rampRegression.add(elapsedTime, robot.getPose().getX(), power);
            Log.d("DriveFeedForwardVel", elapsedTime + "\t" + robot.getPose().getX() + "\t" + power);

            robot.getDrive().setDrivePower(power);
        }
        robot.getDrive().setDrivePower(0);

        RampRegression.RampResult rampResult = rampRegression.fit(true);
        rampRegression.save(LoggingUtil.getLogFile(Misc.formatInvariant(
                "DriveRampRegression-%d.csv", System.currentTimeMillis())));

        Thread.sleep(500);

        double maxPowerTime = DISTANCE / maxVel;
        AccelRegression accelRegression = new AccelRegression();
        startTime = clock.seconds();


        robot.setPose(new Pose2d(0,0,Math.PI));
        robot.getDrive().setDrivePower(-MAX_POWER);
        while (!isStopRequested()) {
            double elapsedTime = clock.seconds() - startTime;
            if (elapsedTime > maxPowerTime) {
                break;
            }
            accelRegression.add(elapsedTime, robot.getPose().getX(), MAX_POWER);
            Log.d("DriveFeedForwardAccel", elapsedTime + "\t" + robot.getPose().getX());

        }
        robot.getDrive().setDrivePower(0);


        AccelRegression.AccelResult accelResult = accelRegression.fit(rampResult.kV, rampResult.kStatic);

        accelRegression.save(LoggingUtil.getLogFile(Misc.formatInvariant(
                "DriveAccelRegression-%d.csv", System.currentTimeMillis())));


        telemetry.addData("kV",rampResult.kV);
        telemetry.addData("kA",accelResult.kA);
        telemetry.addData("kStatic",rampResult.kStatic);
        telemetry.addData("Ramp Correlation",rampResult.rSquare);
        telemetry.addData("Accel Correlation",accelResult.rSquare);
        telemetry.update();
        Log.d("DriveFeedForward", "kV: " + rampResult.kV);
        Log.d("DriveFeedForward", "kA: " + accelResult.kA);
        Log.d("DriveFeedForward", "kStatic: " + rampResult.kStatic);
        Log.d("DriveFeedForward", "Ramp Correlation: " + rampResult.rSquare);
        Log.d("DriveFeedForward", "Accel Correlation: " + accelResult.rSquare);

        while (!isStopRequested()) {
            idle();
        }
    }
}