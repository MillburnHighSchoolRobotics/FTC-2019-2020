package com.millburnrobotics.skystone.test.modes;

import com.millburnrobotics.lib.control.PathBuilder;
import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.lib.geometry.Waypoint;
import com.millburnrobotics.skystone.Constants;
import com.millburnrobotics.skystone.Robot;
import com.millburnrobotics.skystone.auto.AutoModeBase;
import com.millburnrobotics.skystone.auto.actions.drive.DriveFollowPathAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.ArrayList;

@Autonomous(group = "test")
public class FoundationSectionTest extends AutoModeBase {

    @Override
    public void routine() {
        Robot.getInstance().side = Constants.Side.BLUE;

        double claw_to_front = 2.75;
        double claw_extend = 0.5;

        double X_BLUE_DELIVERY = -24-9-claw_extend;
        double Y_BLUE_DELIVERY = 72-4-34.5/2.0+(9-claw_to_front);

        Robot.getInstance().getOdometry().setPose(new Pose(X_BLUE_DELIVERY, Y_BLUE_DELIVERY-8,Math.PI));
        ArrayList<Waypoint> w1 = new ArrayList<>();
        w1.add(new Waypoint(new Pose(0, 0,0),0));
        w1.add(new Waypoint(new Pose(0,72,0),0));
        runAction(new DriveFollowPathAction(PathBuilder.buildPath(w1),0.15,1));
    }
}
