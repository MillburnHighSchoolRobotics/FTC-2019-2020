package com.millburnrobotics.skystone.subsystems;

import com.millburnrobotics.lib.control.Path;
import com.millburnrobotics.lib.followers.PurePursuitFollower;
import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.lib.math.MathUtils;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drive extends Subsystem {
    private PurePursuitFollower follower = null;
    private Path current_path;
    private double strafeThreshold = 2;
    private double rotationThreshold = 2;
    private double kp = 0.014;

    @Override
    public void init(boolean auto) {
        stop();
    }

    @Override
    public void outputToTelemetry(Telemetry telemetry) {
        if (!isDoneWithPath()) {
            telemetry.addData("Dist", ""+follower.getLastOnPath());
        }
    }

    @Override
    public void update() {

    }
    public void setDrivePower(double lfPower, double lbPower, double rfPower, double rbPower) {
        Robot.getInstance().lf.setPower(lfPower);
        Robot.getInstance().lb.setPower(lbPower);
        Robot.getInstance().rf.setPower(-rfPower);
        Robot.getInstance().rb.setPower(-rbPower);
    }
    public void setDrivePower(double[] powers) {
        setDrivePower(powers[0],powers[1],powers[2],powers[3]);
    }
    public void setDrivePower(double power) {
        setDrivePower(power,power,power,power);
    }
    public void setDrivePower(double powerLeft, double powerRight) {
        setDrivePower(powerLeft, powerLeft, powerRight, powerRight);
    }
    public void stop() {
        setDrivePower(0);
    }
    public void vectorTo(Pose currentPose, Pose targetPose, double power) {
        setDrivePower(powerVector(currentPose, targetPose, power));
    }
    private double[] powerVector(Pose currentPose, Pose targetPose, double power) {
        double scale, lf = 0, lb = 0, rf = 0, rb = 0;
        double absoluteAngle = Math.atan2(targetPose.getY() - currentPose.getY(), targetPose.getX() - currentPose.getX());
        double relAngle = Math.toDegrees(MathUtils.normalize(absoluteAngle - currentPose.getHeading()));

        if (relAngle >= 0 && relAngle < 90) {
            scale = MathUtils.tanDegrees(relAngle - 45);
            lf = 1;
            lb = scale;
            rf = scale;
            rb = 1;
        } else if (relAngle >= 90 && relAngle < 180) {
            scale = MathUtils.tanDegrees(relAngle - 135);
            lf = -scale;
            lb = 1;
            rf = 1;
            rb = -scale;
        } else if (relAngle >= 180 && relAngle < 270) {
            scale = MathUtils.tanDegrees(relAngle - 225);
            lf = -1;
            lb = -scale;
            rf = -scale;
            rb = -1;
        } else if (relAngle >= 270 && relAngle < 360) {
            scale = MathUtils.tanDegrees(relAngle - 315);
            lf = scale;
            lb = -1;
            rf = -1;
            rb = scale;
        }
        lf *= power;
        lb *= power;
        rf *= power;
        rb *= power;


        double heading = currentPose.getHeading();
        if (heading - targetPose.getHeading() > 180) {
            heading -= 360;
        } else if (targetPose.getHeading() - heading > 180) {
            heading += 360;
        }
        double u = kp*(targetPose.getHeading()-heading);
        if (MathUtils.equals(targetPose.getHeading(), heading, rotationThreshold)) {
            u = 0;
        }

        lf -= (1-power)*u;
        lb -= (1-power)*u;
        rf += (1-power)*u;
        rb += (1-power)*u;

        return new double[] {lf,lb,rf,rb};
    }
    public void followPath(Path path) {
        this.current_path = path;
        follower = new PurePursuitFollower(current_path);
    }
    public void updatePathFollower(Pose currentPose) {
        Pose nextPose = follower.updatePose(currentPose);
        double power = follower.updatePower();
        vectorTo(currentPose, nextPose, power);
    }

    public boolean isDoneWithPath() {
        return MathUtils.equals(Robot.getInstance().getOdometry().getPose().distTo(current_path.end()),0,strafeThreshold);
    }
}