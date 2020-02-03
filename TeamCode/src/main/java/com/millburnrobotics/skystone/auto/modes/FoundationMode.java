package com.millburnrobotics.skystone.auto.modes;

import com.millburnrobotics.skystone.auto.actions.drive.DriveFollowPathAction;
import com.millburnrobotics.skystone.paths.TestPath;

public class FoundationMode extends AutoModeBase {

    @Override
    public void routine() {
        waitForStart();
        runAction(new DriveFollowPathAction(new TestPath()));
    }
}
