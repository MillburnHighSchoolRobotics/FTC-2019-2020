package com.millburnrobotics.lib.control;

import com.millburnrobotics.lib.math.Pose;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class Path {
    List<PathSegment> segments;
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
        for (PathSegment segment : segments) {
            if ((s-segment.length()) < 0) {
                return segment.get(s);
            }
            s -= segment.length();
        }
        return end(); // s > path.length
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
        return segments.get(0).start();
    }
    public Pose end() {
        return segments.get(segments.size()-1).end();
    }
    public int size() {
        return segments.size();
    }
}