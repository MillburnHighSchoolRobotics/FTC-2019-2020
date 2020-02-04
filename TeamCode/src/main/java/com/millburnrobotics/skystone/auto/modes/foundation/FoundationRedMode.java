package com.millburnrobotics.skystone.auto.modes.foundation;

import com.millburnrobotics.skystone.Constants;
import com.millburnrobotics.skystone.auto.AutoModeBase;
import com.millburnrobotics.skystone.auto.actions.drive.DriveFollowPathAction;
import com.millburnrobotics.skystone.paths.ToFoundationPath;
import com.millburnrobotics.skystone.subsystems.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "Red")
public class FoundationRedMode extends AutoModeBase {

    @Override
    public void routine() {
        Robot.getInstance().side = Constants.SIDE.RED;
        runAction(new DriveFollowPathAction(new ToFoundationPath()));
    }
}
