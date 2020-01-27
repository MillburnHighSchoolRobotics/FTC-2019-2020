package com.millburnrobotics.skystone.test;

import com.millburnrobotics.lib.control.Path;
import com.millburnrobotics.lib.control.PathGenerator;
import com.millburnrobotics.lib.math.Pose;


public class SplineGeneratorTest {
    public static void main(String[] args) {
        PathGenerator generator = new PathGenerator(new Pose(0,0,0));
        Path path = generator.splineTo(new Pose(24,24,0)).generatePath();
        System.out.println("Start: " + path.start());
        System.out.println("End: " + path.end());
        System.out.println("Length: " + path.length());
        for (int x = 0; x < path.length(); x++) {
            System.out.println("Get("+x+"): " + path.get(x));
        }
    }
}
