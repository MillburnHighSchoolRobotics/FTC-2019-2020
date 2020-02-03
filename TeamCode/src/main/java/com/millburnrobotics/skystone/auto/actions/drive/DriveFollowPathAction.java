package com.millburnrobotics.skystone.auto.actions.drive;

import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.skystone.auto.actions.Action;

import java.util.ArrayList;
import java.util.List;

public class DriveFollowPathAction implements Action {

    private ArrayList<Pose> waypoints;

    public DriveFollowPathAction(List<Pose> poses) {
        this.waypoints = new ArrayList<>(poses);

    }


    @Override
    public void start() {

    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void done() {

    }
}
