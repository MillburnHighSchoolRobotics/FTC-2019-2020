package com.millburnrobotics.lib.profile;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import static com.millburnrobotics.skystone.Constants.DriveConstants.LOOK_AHEAD;
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
        double v0 = 45.0;
        length -= LOOK_AHEAD;
        MotionProfile profile = new MotionProfile();

        MotionSegment accelProfile = new MotionSegment(new MotionState(0,v0, maxAccel),(maxVel-v0)/maxAccel);
        MotionSegment tempProfile = new MotionSegment(new MotionState(0,maxVel,-maxAccel),(maxVel-15.0)/maxAccel);
        double dt = (length-tempProfile.end().x-accelProfile.end().x)/accelProfile.end().v;
        MotionSegment constVelProfile = new MotionSegment(new MotionState(accelProfile.end().x,maxVel,0),dt);
<<<<<<< HEAD
        MotionSegment deccelProfile = new MotionSegment(new MotionState(constVelProfile.end().x,maxVel,-maxAccel),(maxVel/maxAccel));
=======
        MotionSegment deccelProfile = new MotionSegment(new MotionState(constVelProfile.end().x,maxVel,-maxAccel),(maxVel-15)/maxAccel);

        if (dt < 0) {

        }

>>>>>>> b1e57df00b059d239940c9a93d4318e59c78eaec
//        if (2*accelProfile.end().x < length) { // 2 do not fit - refactor size
//            double t = 0.5*length/(maxVel); // one ramp up
//            accelProfile = new MotionSegment(new MotionState(0,0,maxAccel),t);
//            MotionSegment deccelProfile = new MotionSegment(new MotionState(length/2.0,accelProfile.end().v,-maxAccel),t);
//
//            profile.segments.add(accelProfile);
//            profile.segments.add(deccelProfile);
//        } else { // mid section
//            double dt = (length-(2*accelProfile.end().x))/accelProfile.end().v;
//            MotionSegment constVelProvile = new MotionSegment(new MotionState(accelProfile.end().x,accelProfile.end().v,0),dt);
//            MotionSegment deccelProfile = new MotionSegment(new MotionState(length-accelProfile.end().x,accelProfile.end().v,-maxAccel),(maxVel/maxAccel));
//
//            profile.segments.add(accelProfile);
//            profile.segments.add(constVelProvile);
//            profile.segments.add(deccelProfile);
//        }


        profile.segments.add(accelProfile);
        profile.segments.add(constVelProfile);
        profile.segments.add(deccelProfile);
        return profile;
    }
}
