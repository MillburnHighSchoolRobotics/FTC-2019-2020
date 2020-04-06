package com.millburnrobotics.lib.profile;

import static com.millburnrobotics.skystone.Constants.DriveConstants.MAX_A;
import static com.millburnrobotics.skystone.Constants.DriveConstants.MAX_J;
import static com.millburnrobotics.skystone.Constants.DriveConstants.MAX_V;

public class MotionProfileGenerator {
    private static double maxVel = MAX_V;
    private static double maxAccel = MAX_A;
    private static double maxJerk = MAX_J;
    private static double v0 = 45.0;
    private static double vf = 15.0;
    public void setMax(double maxVel, double maxAccel, double maxJerk) {
        MotionProfileGenerator.maxVel = maxVel;
        MotionProfileGenerator.maxAccel = maxAccel;
        MotionProfileGenerator.maxJerk = maxJerk;
    }
    public void setMin(double v0, double vf) {
        MotionProfileGenerator.v0 = v0;
        MotionProfileGenerator.vf = vf;
    }
    public static MotionProfile generateProfile(double length) {
        MotionProfile profile = new MotionProfile();

        MotionSegment accelProfile = new MotionSegment(new MotionState(0,v0, maxAccel),(maxVel-v0)/maxAccel);
        MotionSegment tempProfile = new MotionSegment(new MotionState(0,maxVel,-maxAccel),(maxVel-vf)/maxAccel);
        double dt = (length-tempProfile.end().x-accelProfile.end().x)/accelProfile.end().v;

        if (dt < 0) {
            maxVel = Math.sqrt((2*maxAccel*length+v0*v0+vf*vf)/2.0);
            accelProfile = new MotionSegment(new MotionState(0,v0, maxAccel),(maxVel-v0)/maxAccel);
            dt = (length-accelProfile.end().x)/accelProfile.end().v;

            MotionSegment deccelProfile = new MotionSegment(new MotionState(accelProfile.end().x,maxVel,-maxAccel),dt);

            profile.segments.add(accelProfile);
            profile.segments.add(deccelProfile);
        } else {
            MotionSegment constVelProfile = new MotionSegment(new MotionState(accelProfile.end().x,maxVel,0),dt);
            MotionSegment deccelProfile = new MotionSegment(new MotionState(constVelProfile.end().x,maxVel,-maxAccel),(maxVel-15)/maxAccel);

            profile.segments.add(accelProfile);
            profile.segments.add(constVelProfile);
            profile.segments.add(deccelProfile);
        }
        return profile;
    }
}
