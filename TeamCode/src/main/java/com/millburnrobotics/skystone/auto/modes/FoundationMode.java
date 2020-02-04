package com.millburnrobotics.skystone.auto.modes;

import com.millburnrobotics.skystone.auto.actions.drive.DriveFollowPathAction;
import com.millburnrobotics.skystone.paths.ToFoundation;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "Auton")
public class FoundationMode extends AutoModeBase {

    @Override
    public void routine() {
        runAction(new DriveFollowPathAction(new ToFoundation()));
    }
}
