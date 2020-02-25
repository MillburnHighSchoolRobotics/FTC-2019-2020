package com.millburnrobotics.lib.control;

import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.lib.util.MathUtils;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class Path {
    List<PathSegment> segments;
    double startHeading;
    double endHeading;
    public Path() {
        this.segments = new ArrayList<>();
    }
    public Path(PathSegment segment) {
        Collections.fill(segments,segment);
    }
    public Path(List<PathSegment> segments) {
        this.segments = segments;
    }
    public Pose get(double s) {
        double heading = MathUtils.map(s,0,length(),startHeading,endHeading);
        Pose p = null;
        for (PathSegment segment : segments) {
            if (MathUtils.equals(s, segment.length())) {
                s = segment.length();
            }
            if (s <= segment.length()) {
                p = segment.get(s);
                break;
            }
            s -= segment.length();
        }
        if (p == null) {
            p = end(); // s > path.length
        }
        return new Pose(p.x, p.y, heading);
    }
    public Pose _get(double r) {
        double heading = MathUtils.map(r,0,size(),startHeading,endHeading);
        int seg = (int) Math.floor(r);
        return new Pose(segments.get(seg)._get(r-seg).vec(), heading);
    }
    public Pose deriv(double s) {
        for (PathSegment segment : segments) {
            if ((s-segment.length()) < 0) {
                return segment.deriv(s);
            }
            s -= segment.length();
        }
        return segments.get(segments.size()-1).deriv(s);
    }
    public Pose secondDeriv(double s) {
        for (PathSegment segment : segments) {
            if ((s-segment.length()) < 0) {
                return segment.deriv(s);
            }
            s -= segment.length();
        }
        return segments.get(segments.size()-1).secondDeriv(s);
    }
    public void setStartHeading(double h) {
        this.startHeading = h;
    }
    public void setEndHeading(double h) {
        this.endHeading = h;
    }
    public List<PathSegment> segments() {
        return segments;
    }
    public PathSegment segment(int i) {
        return segments.get(i);
    }
    public void add(PathSegment segment) {
        segments.add(segment);
    }
    public double length() {
        double sum = 0;
        for (PathSegment segment : segments) {
            sum += segment.length();
        }
        return sum;
    }
    public PathSegment startSegment() {
        return segments.get(0);
    }
    public PathSegment endSegment() {
        return segments.get(segments.size()-1);
    }
    public Pose start() {
        Pose v = segments.get(0).start();
        return new Pose(v.x, v.y, startHeading);
    }
    public Pose end() {
        Pose v = segments.get(segments.size()-1).end();
        return new Pose(v.x, v.y, endHeading);
    }
    public int size() {
        return segments.size();
    }
}