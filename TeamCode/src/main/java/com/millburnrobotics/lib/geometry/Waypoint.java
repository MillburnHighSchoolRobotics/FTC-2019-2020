package com.millburnrobotics.lib.geometry;

public class Waypoint {
    public Pose pose;
    public double heading; // direction the robot is moving towards
    public String marker;

    public Waypoint(Pose pose, double heading) {
        this.pose = pose;
        this.heading = heading;
    }
    public Waypoint(Pose pose, double heading, String marker) {
        this.pose = pose;
        this.heading = heading;
        this.marker = marker;
    }
}