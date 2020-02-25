package com.millburnrobotics.skystone.test.modes;

import com.millburnrobotics.lib.control.PathBuilder;
import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.lib.geometry.Waypoint;
import com.millburnrobotics.skystone.auto.AutoModeBase;
import com.millburnrobotics.skystone.auto.actions.drive.DriveFollowPathAction;
import com.millburnrobotics.skystone.subsystems.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.ArrayList;

@Autonomous(group = "test")
public class TestMode extends AutoModeBase {

    @Override
    public void routine() {
        ArrayList<Waypoint> w1 = new ArrayList<>();
        w1.add(new Waypoint(new Pose(0, 0,0),0));
        w1.add(new Waypoint(new Pose(0,48,0),0));
        runAction(new DriveFollowPathAction(PathBuilder.buildPath(w1),0.2,0.85));

        while(!isStopRequested() && opModeIsActive()) {
            Robot.getInstance().getDrive().stop();
        }
    }
}
