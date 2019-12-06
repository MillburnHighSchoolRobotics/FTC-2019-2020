package org.firstinspires.ftc.teamcode.robot;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TrajectoryFollower {
    private Trajectory trajectory;
    private ElapsedTime e = new ElapsedTime();

    public void followTrajectory(Trajectory trajectory) {
        this.trajectory = trajectory;
        e.reset();
    }
    public Pose2d update(Pose2d currentPose) {
        Pose2d targetPoseTrajectory = trajectory.get(e.seconds()); // the current pos and target pos use reversed x and y values because of the fps
        Pose2d targetPose = new Pose2d(-targetPoseTrajectory.getY(),targetPoseTrajectory.getX(),targetPoseTrajectory.getHeading());
        Pose2d targetVelocityTrajectory = trajectory.velocity(e.seconds());
        Pose2d targetVelocity = new Pose2d(-targetVelocityTrajectory.getY(),targetVelocityTrajectory.getX(),targetVelocityTrajectory.getHeading());
        Pose2d targetRobotVelocity = new Pose2d(targetVelocity.vec().rotated(-targetPose.getHeading()),targetVelocity.getHeading());
        Log.d("trajectoryfollower","Target Pose: " + targetPose.toString());
        Log.d("trajectoryfollower","Target Velocity: " + targetVelocity.toString());
        Log.d("trajectoryfollower","Target Robot Velocity: " + targetRobotVelocity.toString());
        return targetRobotVelocity;
    }
    public boolean activeTrajectory() {
        return e.seconds() < trajectory.duration();
    }
}
