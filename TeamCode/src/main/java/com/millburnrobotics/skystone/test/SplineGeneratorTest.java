package com.millburnrobotics.skystone.test;

import com.millburnrobotics.lib.control.Path;
import com.millburnrobotics.lib.control.PathBuilder;
import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.lib.geometry.Waypoint;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

public class SplineGeneratorTest {
    public static void main(String[] args) {
        ElapsedTime timer = new ElapsedTime();
        ArrayList<Waypoint> waypoints = new ArrayList<>();
//        waypoints.add(new Waypoint(new Pose(24,0,0),Math.PI));
//        waypoints.add(new Waypoint(new Pose(24,24,0),0));
//        waypoints.add(new Waypoint(new Pose(24,12,0),Math.PI));
//        waypoints.add(new Waypoint(new Pose(18,0,0),3*Math.PI/2));
//        waypoints.add(new Waypoint(new Pose(12,0,0),Math.PI/2));
//        waypoints.add(new Waypoint(new Pose(36,12,0),Math.PI/4));
//        waypoints.add(new Waypoint(new Pose(18,18,0),5*Math.PI/4));
//        waypoints.add(new Waypoint(new Pose(12,6,0),5*Math.PI/4));
//        waypoints.add(new Waypoint(new Pose(0,0,0),Math.PI));
        waypoints.add(new Waypoint(new Pose(0,0,0),0));
        waypoints.add(new Waypoint(new Pose(48,48,0),0));
        Path path =  PathBuilder.buildPath(waypoints, 0.1, 0.8, 0.9);

        System.out.println("Start: " + path.start());
        System.out.println("End: " + path.end());
        System.out.println("Length: " + path.length() + "\n");

        for (int x = 0; x < path.length(); x++) {
//            System.out.println("(" + path.get(x).x + "," + path.get(x).y + ")");
            System.out.println(path.getPower(x));
        }

        System.out.println("\nTime: " + timer.milliseconds());
    }
}
