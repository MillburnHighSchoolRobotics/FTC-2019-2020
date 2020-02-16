package com.millburnrobotics.skystone.test;

import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.skystone.Constants;
import com.millburnrobotics.skystone.auto.AutoModeBase;
import com.millburnrobotics.skystone.auto.actions.drive.DriveToPoseAction;
import com.millburnrobotics.skystone.subsystems.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "Test")
public class ResetMode extends AutoModeBase {

    @Override
    public void routine() {
        Robot.getInstance().side = Constants.Side.BLUE;
        double pref_x = Double.valueOf(Robot.getInstance().readPreference("x"));
        double pref_y = Double.valueOf(Robot.getInstance().readPreference("y"));
        double pref_heading = Double.valueOf(Robot.getInstance().readPreference("heading"));
        Robot.getInstance().getOdometry().setPose(new Pose(pref_x, pref_y, pref_heading));

        runAction(new DriveToPoseAction(new Pose(-72+Constants.DriveConstants.BOT_WIDTH/2.0,-48+Constants.DriveConstants.BOT_LENGTH/2.0,0), 0.6));
    }
}
