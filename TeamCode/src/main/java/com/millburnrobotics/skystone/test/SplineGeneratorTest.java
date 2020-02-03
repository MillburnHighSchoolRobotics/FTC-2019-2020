package com.millburnrobotics.skystone.test;

import com.millburnrobotics.lib.control.Path;
import com.millburnrobotics.lib.control.PathBuilder;
import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.lib.geometry.Waypoint;

import java.util.ArrayList;

public class SplineGeneratorTest {
    public static void main(String[] args) {
        ArrayList<Waypoint> waypoints = new ArrayList<>();
        waypoints.add(new Waypoint(new Pose(0,0,0),0));
        waypoints.add(new Waypoint(new Pose(24,24,90),0));
        Path path =  PathBuilder.buildPath(waypoints, 1, 0.4);

        System.out.println("Start: " + path.start());
        System.out.println("End: " + path.end());
        System.out.println("Length: " + path.length());
        for (int x = 0; x < path.length(); x++) {
            System.out.println("Get("+x+"): " + path.get(x) + "\t" + path.getPower(x));
        }
    }
}
