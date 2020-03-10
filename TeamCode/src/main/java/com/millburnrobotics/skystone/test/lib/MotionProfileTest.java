package com.millburnrobotics.skystone.test.lib;

import com.millburnrobotics.lib.profile.MotionProfile;
import com.millburnrobotics.lib.profile.MotionProfileGenerator;
import com.millburnrobotics.lib.util.MathUtils;

import static com.millburnrobotics.skystone.Constants.DriveConstants.MAX_V;

public class MotionProfileTest {
    public static void main(String[] args) {
        double len = 72;
        MotionProfile profile = MotionProfileGenerator.generateProfile(len);
        System.out.println(profile.duration());
        for (double i = 0; i < len; i++) {
            double t = MathUtils.map(i, 0, len, 0, profile.duration());

            double kv = 1.0/MAX_V;
            double ka = 0.002;
            double power = kv*profile.get(t).v+ka*profile.get(t).a;

            System.out.println("("+t+","+power+")");
        }
    }
}
