package com.millburnrobotics.skystone.auto.modes.park;

import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.skystone.Constants;
import com.millburnrobotics.skystone.Robot;
import com.millburnrobotics.skystone.auto.AutoModeBase;
import com.millburnrobotics.skystone.auto.actions.drive.DriveToPoseAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "park")
public class ParkModeBridgeRight extends AutoModeBase {

    @Override
    public void routine() {
        Robot.getInstance().side = Constants.Side.BLUE;
        Robot.getInstance().getOdometry().setPose(new Pose(0, 0, 0));
        runAction(new DriveToPoseAction(new Pose(0, 24, 0), 0.5));
        runAction(new DriveToPoseAction(new Pose(24, 24, 0), 0.5));
    }
}
