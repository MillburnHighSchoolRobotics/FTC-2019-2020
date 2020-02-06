package com.millburnrobotics.lib.geometry;

public class Waypoint {
    public Pose pose;
    public double bearing; // direction the robot is moving towards
    public String marker;

    public Waypoint(Pose pose, double bearing) {
        this.pose = pose;
        this.bearing = bearing;
    }
    public Waypoint(Pose pose, double bearing, String marker) {
        this.pose = pose;
        this.bearing = bearing;
        this.marker = marker;
    }
    Waypoint(Waypoint w) {
        this.pose = w.pose;
        this.bearing = w.bearing;
        this.marker = w.marker;
    }
}