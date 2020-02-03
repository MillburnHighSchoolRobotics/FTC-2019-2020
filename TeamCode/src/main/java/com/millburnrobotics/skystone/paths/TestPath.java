package com.millburnrobotics.skystone.paths;

import com.millburnrobotics.lib.control.Path;
import com.millburnrobotics.lib.control.PathBuilder;
import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.lib.geometry.Waypoint;

import java.util.ArrayList;


public class TestPath implements PathContainer {

    @Override
    public Path buildPath() {
        ArrayList<Waypoint> waypoints = new ArrayList<>();

//        waypoints.add(new Waypoint(start));
        waypoints.add(new Waypoint(new Pose(24,24,90),0));

        return PathBuilder.buildPath(waypoints, 1);
    }
}
