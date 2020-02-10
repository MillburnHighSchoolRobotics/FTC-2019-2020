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
    @Override
    public String toString() {
        return ("("+Math.round(pose.x*1000.0)/1000.0+","+Math.round(pose.y*1000.0)/1000.0+","+Math.round(Math.toDegrees(bearing)*1000.0)/1000.0+")");
    }
}