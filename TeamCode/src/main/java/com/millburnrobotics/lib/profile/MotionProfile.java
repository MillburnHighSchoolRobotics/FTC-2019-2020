package com.millburnrobotics.lib.profile;

import com.millburnrobotics.lib.util.MathUtils;


import java.util.ArrayList;
import java.util.List;

public class MotionProfile {
    List<MotionSegment> segments;
    public MotionProfile() {
        segments = new ArrayList<>();
    }
    public MotionState get(double t) {
        double rt = MathUtils.clamp(t, 0, duration());
        for (MotionSegment segment : segments) {
            if (rt <= segment.dt) {
                return segment.get(rt);
            }
            rt -= segment.dt;
        }
        if (segments.size() == 0) {
            return new MotionState();
        }
        return segments.get(segments.size()-1).end();
    }
    public double duration() {
        double duration = 0;
        for (MotionSegment segment : segments) {
            duration += segment.dt;
        }
        return duration;
    }
    public void start() {
        get(0);
    }
    public void end() {
        get(duration());
    }
}
