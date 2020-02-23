package com.millburnrobotics.skystone.test.modes;

import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.skystone.Constants;
import com.millburnrobotics.skystone.auto.AutoModeBase;
import com.millburnrobotics.skystone.subsystems.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "Test")
public class SkystoneDetectionRedMode extends AutoModeBase {

    @Override
    public void routine() {
        Robot.getInstance().getOdometry().setPose(new Pose(0,0,0));

        Robot.getInstance().side = Constants.Side.RED;
        Robot.getInstance().getCamera().detectBlock();

        while(!isStopRequested() && opModeIsActive());
    }
}
