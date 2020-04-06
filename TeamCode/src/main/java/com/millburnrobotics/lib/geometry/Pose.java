package com.millburnrobotics.lib.geometry;

import com.millburnrobotics.lib.util.MathUtils;

import static com.millburnrobotics.skystone.Constants.DriveConstants.PURE_PURSUIT_THRESH;

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
    public Pose (Pose p, double heading) {
        this.x = p.x;
        this.y = p.y;
        this.heading = heading;
    }
    public Pose(double[] vector, double heading) {
        this.x = vector[0];
        this.y = vector[1];
        this.heading = heading;
    }
    public Pose(double[] vector) {
        this.x = vector[0];
        this.y = vector[1];
        this.heading = 0;
    }
    public Pose() {
        this.x = 0;
        this.y = 0;
        this.heading = 0;
    }

    Pose (Pose p) {
        this.x = p.x;
        this.y = p.y;
        this.heading = p.heading;
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
    public double[] vec() {
        return new double[] {x,y};
    }
    public double norm() {
        return Math.sqrt(x*x+y*y);
    }
    public Pose plus(Pose p) {
        return new Pose(x+p.x,y+p.y, MathUtils.normalize(heading+p.heading));
    }
    public Pose minus(Pose p) {
        return new Pose(x-p.x,y-p.y,MathUtils.normalize(heading-p.heading));
    }
    public Pose times(Pose p) {
        return new Pose(x*p.x,y*p.y,MathUtils.normalize(heading*p.heading));
    }
    public Pose div(Pose p) {
        return new Pose(x/p.x,y/p.y,MathUtils.normalize(heading/p.heading));
    }
    public Pose times(double k) {
        return new Pose(x*k,y*k,MathUtils.normalize(heading*k));
    }
    public Pose div(double k) {
        return new Pose(x/k,y/k,MathUtils.normalize(heading/k));
    }
    public double atan() {
        return Math.atan2(y,x);
    }
    public double distTo(Pose p) {
        return this.minus(p).norm();
    }
    public double dot(Pose p) {
        return(x*p.x+y*p.y);
    }
    public Pose rotate(double theta) {
        double x1 = x*Math.cos(theta)-y*Math.sin(theta);
        double y1 = x*Math.sin(theta)+y*Math.cos(theta);
        double heading1 = MathUtils.normalize(heading+theta);
        return new Pose(x1,y1,heading1);
    }
    public Pose polar(double r) {
        return new Pose(r*Math.cos(heading),r*Math.sin(heading));
    }
    public double normalize() {
        return MathUtils.normalize(heading);
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
    public  Pose relDistanceToTarget(Pose target) {
        double distance = target.distTo(this);
        double relAngle = MathUtils.normalize(target.minus(this).atan() - heading);
        return new Pose(polar(distance),relAngle).rotate(-Math.PI/2);
    }

    @Override
    public String toString() {
        return ("("+Math.round(x*1000.0)/1000.0+","+Math.round(y*1000.0)/1000.0+","+Math.round(Math.toDegrees(heading)*1000.0)/1000.0+")");
    }
    @Override
    public boolean equals(Object p1) {
        return (distTo((Pose)p1) <= PURE_PURSUIT_THRESH);
    }
}