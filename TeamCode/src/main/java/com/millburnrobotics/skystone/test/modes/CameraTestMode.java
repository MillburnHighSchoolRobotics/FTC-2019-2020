package com.millburnrobotics.skystone.test.modes;

import com.millburnrobotics.skystone.Constants;
import com.millburnrobotics.skystone.auto.AutoModeBase;
import com.millburnrobotics.skystone.subsystems.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "Test")
public class CameraTestMode extends AutoModeBase {

    @Override
    public void routine() {
        Robot.getInstance().side = Constants.Side.BLUE;
        Robot.getInstance().getCamera().detectBlock();
        while(!isStopRequested() && opModeIsActive());
    }
}
