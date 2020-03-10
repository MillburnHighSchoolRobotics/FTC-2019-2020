package com.millburnrobotics.skystone.test;

import android.util.Log;

import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.lib.util.MathUtils;
import com.millburnrobotics.skystone.Constants;
import com.millburnrobotics.skystone.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(group = "test")
public class FeedforwardTuner extends LinearOpMode {
    public static final double MAX_POWER = 1;
    public static final double DISTANCE = 100;

    @Override
    public void runOpMode() { //http://www.alcula.com/calculators/statistics/correlation-coefficient/

        Robot.getInstance().init(hardwareMap, true);


        waitForStart();

        if (isStopRequested()) return;

        try {
            ka();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        while (!isStopRequested()) {
            idle();
        }
    }
    public void kv() throws InterruptedException {
        ElapsedTime clock = new ElapsedTime();

        double maxVel = Constants.DriveConstants.rpmToVelocity(325);
        double finalVel = MAX_POWER * maxVel;
        double accel = (finalVel * finalVel) / (2.0 * DISTANCE);
        double rampTime = Math.sqrt(2.0 * DISTANCE / accel);

        double startTime = clock.seconds();

        Robot.getInstance().getOdometry().setPose(new Pose());
        while (!isStopRequested()) {
            double elapsedTime = clock.seconds() - startTime;
            if (elapsedTime > rampTime) {
                break;
            }
            double vel = accel * elapsedTime;
            double power = vel / maxVel;
            Log.d("RampRegression", Robot.getInstance().getOdometry().getVelocityY() + " " + power);

            Robot.getInstance().getDrive().setDrivePower(power);
            Robot.getInstance().getOdometry().update();
        }
        Robot.getInstance().getDrive().stop();
    }
    public void ka() throws InterruptedException {
        ElapsedTime clock = new ElapsedTime();

        double maxVel = Constants.DriveConstants.rpmToVelocity(325);
        double maxPowerTime = DISTANCE / maxVel;

        double startTime = clock.seconds();

        Robot.getInstance().getOdometry().setPose(new Pose());
        Robot.getInstance().getDrive().setDrivePower(1);
        double prevTime = 0;
        double prevVel = 0;
        while (!isStopRequested()) {
            double elapsedTime = clock.seconds() - startTime;
            if (elapsedTime > maxPowerTime) {
                break;
            }

            double t = clock.seconds();
            double v = Robot.getInstance().getOdometry().getVelocityY();
            double power =  (v) * Robot.getInstance().getDrive().kv;
            power += (MathUtils.sgn(power) * Robot.getInstance().getDrive().ks);

            double predpower = 1-power;
            double a = (v-prevVel)/(t-prevTime);

            Log.d("AccelRegression", a + " " + predpower);

            prevTime = t;
            prevVel = v;
            Robot.getInstance().getOdometry().update();
            Thread.sleep(5);
        }
        Robot.getInstance().getDrive().stop();
    }
}