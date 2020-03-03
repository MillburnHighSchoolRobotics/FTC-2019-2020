package com.millburnrobotics.skystone.test.lib;

import com.millburnrobotics.lib.control.Path;
import com.millburnrobotics.lib.control.PathBuilder;
import com.millburnrobotics.lib.followers.AdaptivePurePursuitFollower;
import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.lib.geometry.Waypoint;

import java.util.ArrayList;

public class PurePursuitTest {
    public static void main(String[] args) throws InterruptedException {
        ArrayList<Waypoint> w3 = new ArrayList<>(); // cycle 2
        w3.add(new Waypoint(new Pose(-24-9-0.5, 72-4-34.5/2.0+(9-3.5)+2,Math.PI),Math.toRadians(150)));
        w3.add(new Waypoint(new Pose(-40,0,Math.PI),Math.PI));
        w3.add(new Waypoint(new Pose(-24-9-0.5-4,-24-4+(9-3.5)-8,Math.PI),Math.toRadians(215)));
        Path path =  PathBuilder.buildPath(w3);
        AdaptivePurePursuitFollower follower = new AdaptivePurePursuitFollower(path);

        Pose p = new Pose(-24-9-0.5, 72-4-34.5/2.0+(9-3.5)+2,Math.PI);
        while (!p.equals(path.end())) {
            Pose pose1 = follower.getLookaheadPoint(p);
            System.out.println("(x-" + p.x + ")^{2}+(y-" + p.y + ")^{2}=1");
            if (pose1.equals(path.end())) {
                break;
            }
            Thread.sleep(5);
            p = pose1;
        }
    }
}
