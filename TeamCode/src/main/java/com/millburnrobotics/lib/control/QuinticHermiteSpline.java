package com.millburnrobotics.lib.control;

import com.millburnrobotics.lib.math.MathUtils;
import com.millburnrobotics.lib.math.Pose;

import java.util.ArrayList;
import java.util.List;

public class QuinticHermiteSpline extends PathSegment {
    private static final double maxDeltaK = 0.01;
    private static final double maxSegmentLength = 0.25;

    private double x0, x1, dx0, dx1, ddx0, ddx1, y0, y1, dy0, dy1, ddy0, ddy1;
    private double ax, bx, cx, dx, ex, fx, ay, by, cy, dy, ey, fy;

    private double length = 0;

    private List<Double> tSamples = new ArrayList<>();
    private List<Double> sSamples = new ArrayList<>();

    public QuinticHermiteSpline(Pose p0, Pose p1) {
        double scale = 1.2*p0.distTo(p1);
        x0 = p0.x;
        x1 = p1.x;
        dx0 = p0.cos()*scale;
        dx1 = p1.cos()*scale;
        ddx0 = 0;
        ddx1 = 0;
        y0 = p0.y;
        y1 = p1.y;
        dy0 = p0.sin()*scale;
        dy1 = p1.sin()*scale;
        ddy0 = 0;
        ddy1 = 0;

        computeCoefficients();
        parametrize(0,1);
    }

    private void computeCoefficients() {
        ax = -6*x0-3*dx0-0.5*ddx0+0.5*ddx1-3*dx1+6*x1;
        bx = 15*x0+8*dx0+1.5*ddx0-ddx1+7*dx1-15*x1;
        cx = -10*x0-6*dx0-1.5*ddx0+0.5*ddx1-4*dx1+10*x1;
        dx = 0.5*ddx0;
        ex = dx0;
        fx = x0;

        ay = -6*y0-3*dy0-0.5*ddy0+0.5*ddy1-3*dy1+6*y1;
        by = 15*y0+8*dy0+1.5*ddy0-ddy1+7*dy1-15*y1;
        cy = -10*y0-6*dy0-1.5*ddy0+0.5*ddy1-4*dy1+10*y1;
        dy = 0.5*ddy0;
        ey = dy0;
        fy = y0;
    }

    private Pose _get(double t) {
        double x = ax*t*t*t*t*t+bx*t*t*t*t+cx*t*t*t+dx*t*t+ex*t+fx;
        double y = ay*t*t*t*t*t+by*t*t*t*t+cy*t*t*t+dy*t*t+ey*t+fy;
        return new Pose(x,y);
    }

    private void parametrize(double tLow, double tHigh) {
        Pose pLow = _get(tLow);
        Pose pHigh = _get(tHigh);

        double tMid = (tLow+tHigh)/2.0;
        Pose pMid = _get(tMid);

        double deltaK = Math.abs(curvature(tLow)-curvature(tHigh));
        double segmentLength = approxLength(pLow, pMid, pHigh);

        if (deltaK > maxDeltaK || segmentLength > maxSegmentLength) {
            parametrize(tLow, tMid);
            parametrize(tMid, tHigh);
        } else {
            length += segmentLength;
            sSamples.add(length);
            tSamples.add(tHigh);
        }
    }
    private double approxLength(Pose p1, Pose p2, Pose p3) {
        Pose w1 = (p2.minus(p1)).times(2.0);
        Pose w2 = (p2.minus(p3)).times(2.0);

        double det = w1.x*w2.y-w2.x*w1.y;
        double chord = p1.distTo(p3);
        if (MathUtils.equals(det,0)) {
            return chord;
        } else {
            double x1 = p1.x*p1.x+p1.y*p1.y;
            double x2 = p2.x*p2.x+p2.y*p2.y;
            double x3 = p3.x*p3.x+p3.y*p3.y;
            double y1 = x2-x1;
            double y2 = x2-x3;
            Pose origin = new Pose(y1*w2.y-y2*w1.y,y2*w1.x-y1*w2.x);
            origin = origin.div(det);
            double radius = origin.distTo(p1);
            return 2.0*radius*Math.asin(chord/(2.0*radius));
        }
    }

    public double reparam(double s) {
        if (s <= 0.0) return 0;
        if (s >= length) return 1.0;

        int low = 0;
        int high = sSamples.size();
        while (low <= high) {
            int mid = (int)Math.round((low+high)/2.0);

            if (s < sSamples.get(mid)) {
                high = mid-1;
            } else if (s > sSamples.get(mid)) {
                low = mid+1;
            } else {
                return tSamples.get(mid);
            }
        }
        return tSamples.get(low)+(s-sSamples.get(low))*(sSamples.get(high)-tSamples.get(low))/(sSamples.get(high)-sSamples.get(low));
    }

    @Override
    public Pose get(double s) {
        return _get(reparam(s));
    }

    @Override
    public double length() {
        return length;
    }

    @Override
    public Pose deriv(double s) {
        double t = reparam(s);
        Pose deriv = new Pose(dx(t),dy(t));
        double k = 1.0/Math.sqrt(dx(t)*dx(t)+dy(t)+dy(t));
        return deriv.times(k);
    }
    @Override
    public Pose secondDeriv(double s) {
        double t = reparam(s);
        Pose deriv = new Pose(dx(t),dy(t));
        Pose secondDeriv = new Pose(ddx(t),ddy(t));
        double k1 = 1.0/Math.sqrt(dx(t)*dx(t)+dy(t)+dy(t));
        double k2 = (-dx(t)*ddx(t)+dy(t)+ddy(t))/((dx(t)*dx(t)+dy(t)+dy(t))*(dx(t)*dx(t)+dy(t)+dy(t)));
        return secondDeriv.times(k1*k1).plus(deriv.times(k2));
    }


    private double dx(double t) {
        return 5*ax*t*t*t*t+4*bx*t*t*t+3*cx*t*t+2*dx*t+ex;
    }

    private double dy(double t) {
        return 5*ay*t*t*t*t+4*by*t*t*t+3*cy*t*t+2*dy*t+ey;
    }

    private double ddx(double t) {
        return 20*ax*t*t*t+12*bx*t*t+6*cx*t+2*dx;
    }

    private double ddy(double t) {
        return 20*ay*t*t*t+12*by*t*t+6*cy*t+2*dy;
    }

    public double getVelocity(double t) {
        return Math.hypot(dx(t), dy(t));
    }

    public double curvature(double t) {
        return (dx(t)*ddy(t)-ddx(t)*dy(t))/((dx(t)*dx(t)+dy(t)*dy(t))*Math.sqrt((dx(t)*dx(t)+dy
                (t)*dy(t))));
    }
}