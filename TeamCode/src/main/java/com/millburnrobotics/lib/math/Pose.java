package com.millburnrobotics.lib.math;

public class Pose {
    public double x = 0;
    public double y = 0;
    public double heading = 0;
    public Pose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }
    public Pose(double x, double y) {
        this.x = x;
        this.y = y;
        this.heading = 0;
    }
    public Pose() {
        this.x = 0;
        this.y = 0;
        this.heading = 0;
    }
    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }
    public double getHeading() {
        return heading;
    }
    public double norm() {
        return Math.sqrt(x*x+y*y);
    }
    public Pose plus(Pose p) {
        return new Pose(x+p.x,y+p.y,MathUtils.normalize(heading+p.heading));
    }
    public Pose minus(Pose p) {
        return new Pose(x-p.x,y-p.y,MathUtils.normalize(heading+p.heading));
    }
    public Pose times(double k) {
        return new Pose(x*k,y*k,MathUtils.normalize(heading*k));
    }
    public Pose div(double k) {
        return new Pose(x/k,y/k,MathUtils.normalize(heading/k));
    }
    public double distTo(Pose p) {
        return this.minus(p).norm();
    }
    public double dot(Pose p) {
        return(x*p.x+y*p.y);
    }
    public void rotate(double theta) {
        double x1 = x*Math.cos(theta)-y*Math.sin(theta);
        double y1 = x*Math.sin(theta)+y*Math.cos(theta);
        x = x1;
        y = y1;
    }
    public Pose polar(double r) {
        return new Pose(r*Math.cos(heading),r*Math.sin(heading));
    }
    private void normalize() {
        MathUtils.normalize(heading);
    }
    public void setPose(Pose p) {
        this.x = p.x;
        this.y = p.y;
        this.heading = p.heading;
    }
    public double cos() {
        return Math.cos(heading);
    }
    public double sin() {
        return Math.sin(heading);
    }

    @Override
    public String toString() {
        return ("("+x+","+y+","+heading+")");
    }
}