package com.millburnrobotics.skystone.test.modes;

import com.millburnrobotics.skystone.Constants;
import com.millburnrobotics.skystone.auto.AutoModeBase;
import com.millburnrobotics.skystone.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "test")
public class SkystoneDetectionBlueMode extends AutoModeBase {

    @Override
    public void routine() {
        Robot.getInstance().side = Constants.Side.BLUE;
        Robot.getInstance().getCamera().detectBlock();

        while(!isStopRequested() && opModeIsActive());
    }
}
