package org.firstinspires.ftc.teamcode.profile.geometry;

public class Vector2d {
    protected double x;
    protected double y;

    public Vector2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double norm() {
        return Math.hypot(x, y);
    }
    public Vector2d inverse() {
        return new Vector2d(-x, -y);
    }
    public Vector2d plus(Vector2d p) {
        return new Vector2d(x+p.x(), y+p.y());
    }
    public Vector2d minus(Vector2d p) {
        return new Vector2d(x-p.x(), y-p.y());
    }
    public Vector2d times(double c) {
        return new Vector2d(c*x, c*y);
    }
    public Vector2d divide(double c) {
        return new Vector2d(x/c, y/c);
    }

    public double distance(Vector2d p) {
        return minus(p).norm();
    }


    public double x() {
        return x;
    }
    public double y() {
        return y;
    }
}
