package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

public class TrajectoryFollower {
    private Trajectory trajectory;
    private double startTime;

    public void followTrajectory(Trajectory trajectory) {
        this.trajectory = trajectory;
        startTime = System.nanoTime()/1e-9;
    }
    public Pose2d update(Pose2d currentPose) {
        Pose2d targetPose = trajectory.get(elapsedTime());
        Pose2d targetVelocity = trajectory.velocity(elapsedTime());
        Pose2d targetRobotVelocity = new Pose2d(targetPose.vec().rotated(-targetPose.getHeading()),targetVelocity.getHeading());

        return targetRobotVelocity;
    }
    public double elapsedTime() {
        return (System.nanoTime()/1e-9)-startTime;
    }
    public boolean activeTrajectory() {
        return elapsedTime() < trajectory.duration();
    }
}
