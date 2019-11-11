package org.firstinspires.ftc.teamcode.test.roadrunner;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.robot.MohanBot;
import org.firstinspires.ftc.teamcode.threads.ThreadManager;

import static org.firstinspires.ftc.teamcode.robot.GlobalConstants.ROBOT_WIDTH;

/*
 * This routine determines the effective track width. The procedure works by executing a point turn
 * with a given angle and measuring the difference between that angle and the actual angle (as
 * indicated by an external IMU/gyro, track wheels, or some other localizer). The quotient
 * given angle / actual angle gives a multiplicative adjustment to the estimated track width
 * (effective track width = estimated track width * given angle / actual angle). The routine repeats
 * this procedure a few times and averages the values for additional accuracy. Note: a relatively
 * accurate track width estimate is important or else the angular constraints will be thrown off.
 */
@Config
@Autonomous(group = "drive")
public class TrackWidthTuner extends LinearOpMode {
    public static double ANGLE = Math.toRadians(180);
    public static int NUM_TRIALS = 30;

    @Override
    public void runOpMode() throws InterruptedException {
        MohanBot robot = new MohanBot(hardwareMap,this);

        telemetry.log().add("Press play to begin the track width tuner routine");
        telemetry.log().add("Make sure your robot has enough clearance to turn smoothly");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        telemetry.log().clear();
        telemetry.log().add("Running...");
        telemetry.update();
        double lastHeading = 0;

        MovingStatistics trackWidthStats = new MovingStatistics(NUM_TRIALS);
        for (int i = 0; i < NUM_TRIALS; i++) {
            robot.setPose(new Pose2d());

            // it is important to handle heading wraparounds
            double headingAccumulator = 0;

            robot.turn(ANGLE);

            while (!isStopRequested() && robot.isBusy()) {
                double heading = Math.toRadians(ThreadManager.getInstance().getValue("orientation", Double.class));

                headingAccumulator += Math.abs(heading - lastHeading);
                lastHeading = heading;
                robot.update();
            }
            Log.d("mohan finished", "" + Math.toDegrees(Math.abs(headingAccumulator)));

            double trackWidth = ROBOT_WIDTH * ANGLE / Math.abs(headingAccumulator);
            Log.d("eee", "" + trackWidth);
            telemetry.log().add(String.valueOf(trackWidth));
            telemetry.update();
            trackWidthStats.add(trackWidth);
            sleep(1000);
        }

        telemetry.log().clear();
        telemetry.log().add("Tuning complete");
        telemetry.log().add(Misc.formatInvariant("Effective track width = %.2f (SE = %.3f)",
                trackWidthStats.getMean(),
                trackWidthStats.getStandardDeviation() / Math.sqrt(NUM_TRIALS)));
        telemetry.update();

        Log.d("eeee", "" + trackWidthStats.getMean());
    }
}
