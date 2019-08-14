package org.firstinspires.ftc.teamcode.profile.geometry;

public class Pose2d {
    protected double x;
    protected double y;
    protected double theta;

    public Pose2d(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }
    public Pose2d(Vector2d vec, double theta) {
        this.x = vec.x();
        this.y = vec.y();
        this.theta = theta;
    }

    public Pose2d inverse() {
        return new Pose2d(-x, -y, -theta);
    }
    public double norm() {
        return Math.hypot(x, y);
    }
    public Pose2d plus(Pose2d p) {
        return new Pose2d(x+p.x(), y+p.y(), theta+p.theta());
    }
    public Pose2d minus(Pose2d p) {
        return new Pose2d(x-p.x(), y-p.y(), theta-p.theta());
    }
    public Pose2d times(double c) {
        return new Pose2d(c*x, c*y, c*theta);
    }
    public Pose2d divide(double c) {
        return new Pose2d(x/c, y/c, theta/c);
    }

    public boolean equals(Pose2d p) {
        if (x==p.x() && y==p.y() && theta==p.theta()) {
            return true;
        } else {
            return false;
        }
    }

    public Vector2d vec() {
        return new Vector2d(x,y);
    }


    public double x() {
        return x;
    }
    public double y() {
        return y;
    }
    public double theta() {
        return theta;
    }
}
