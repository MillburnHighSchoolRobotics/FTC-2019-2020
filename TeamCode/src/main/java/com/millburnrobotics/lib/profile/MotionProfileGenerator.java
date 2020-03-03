package com.millburnrobotics.lib.profile;

import static com.millburnrobotics.skystone.Constants.DriveConstants.MAX_A;
import static com.millburnrobotics.skystone.Constants.DriveConstants.MAX_J;
import static com.millburnrobotics.skystone.Constants.DriveConstants.MAX_V;

public class MotionProfileGenerator {
    private static double maxVel = MAX_V;
    private static double maxAccel = MAX_A;
    private static double maxJerk = MAX_J;
    public void setMax(double maxVel, double maxAccel, double maxJerk) {
        MotionProfileGenerator.maxVel = maxVel;
        MotionProfileGenerator.maxAccel = maxAccel;
        MotionProfileGenerator.maxJerk = maxJerk;
    }
    public static MotionProfile generateProfile(double length) {
        MotionProfile profile = new MotionProfile();

        double maxV = Math.sqrt(2*maxAccel*length)/2.0;
        if (maxV > maxVel) maxV = maxVel;
        MotionSegment accelProfile = new MotionSegment(new MotionState(0,0,maxAccel),(maxV)/maxAccel);
        MotionSegment deccelProfile = new MotionSegment(new MotionState(length-accelProfile.end().x,maxV,-maxAccel),(maxV)/maxAccel);

        profile.segments.add(accelProfile);
        if (maxV == maxVel) {
            double dt = (length-(2*accelProfile.end().x))/maxVel;
            MotionSegment constVelProvile = new MotionSegment(new MotionState(accelProfile.end().x,maxVel,0),dt);
            profile.segments.add(constVelProvile);
        }
        profile.segments.add(deccelProfile);

        return profile;
    }
}
