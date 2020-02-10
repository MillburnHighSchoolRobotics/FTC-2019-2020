package com.millburnrobotics.skystone.test.lib;

import com.millburnrobotics.lib.control.Path;
import com.millburnrobotics.lib.control.PathBuilder;
import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.lib.geometry.Waypoint;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

public class SplineGeneratorTest {
    public static void main(String[] args) {
        ElapsedTime timer = new ElapsedTime();
        double buildTime, sTime, tTime;
        double s1 = 0, t1 = 0;

        ArrayList<Waypoint> waypoints = new ArrayList<>();
        waypoints.add(new Waypoint(new Pose(24,0,0),Math.PI));
        waypoints.add(new Waypoint(new Pose(24,24,0),0));
        waypoints.add(new Waypoint(new Pose(24,12,0),Math.PI));
        waypoints.add(new Waypoint(new Pose(18,0,0),3*Math.PI/2));
        waypoints.add(new Waypoint(new Pose(12,0,0),Math.PI/2));
        waypoints.add(new Waypoint(new Pose(36,12,0),Math.PI/4));
        waypoints.add(new Waypoint(new Pose(18,18,0),5*Math.PI/4));
        waypoints.add(new Waypoint(new Pose(12,6,0),5*Math.PI/4));
        waypoints.add(new Waypoint(new Pose(0,0,0),Math.PI));
//        waypoints.add(new Waypoint(new Pose(0,0,0),0));
//        waypoints.add(new Waypoint(new Pose(48,48,0),0));

        timer.reset();
        Path path =  PathBuilder.buildPath(waypoints, 0.1, 0.8, 0.9);
        buildTime = timer.milliseconds();

        System.out.println("Start: " + path.start());
        System.out.println("End: " + path.end());
        System.out.println("Length: " + path.length() + "\n");

        double inc1 = .1;
        double inc2 = 1.0/(((1.0/inc1)*path.length())/(waypoints.size()-1));
        timer.reset();

        for (double x = 0; x < path.length(); x+=inc1) {
            Pose p = path.get(x);
//            System.out.println("(" + path.get(x).x + "," + path.get(x).y + ")");
            s1++;
//            System.out.println(path.getPower(x));
        }
        sTime = timer.milliseconds();
        timer.reset();

        for (double x = 0; x < waypoints.size()-1; x+=inc2) {
            Pose p = path._get(x);
//            System.out.println("(" + path._get(x).x + "," + path._get(x).y + ")");
            t1++;
        }
        tTime = timer.milliseconds();
        timer.reset();

        System.out.println("Build Time: " + buildTime);
        System.out.println("S Time: " + sTime);
        System.out.println("T Time: " + tTime);
        System.out.println("\nS Count: " + s1);
        System.out.println("T Count: " + t1);
    }
}
