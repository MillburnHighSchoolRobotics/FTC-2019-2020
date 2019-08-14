package org.firstinspires.ftc.teamcode.profile.trajectory;

import org.firstinspires.ftc.teamcode.profile.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.threads.ThreadManager;

public class TrajectoryBuilder {
    public TrajectoryBuilder moveTo(Pose2d endPose) {
        Pose2d startPose = ThreadManager.getInstance().getValue("pose2d", Pose2d.class);
        return this;
    }
}
