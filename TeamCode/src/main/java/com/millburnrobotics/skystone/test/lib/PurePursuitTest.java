package com.millburnrobotics.skystone.test.lib;

import com.millburnrobotics.lib.control.Path;
import com.millburnrobotics.lib.control.PathBuilder;
import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.lib.geometry.Waypoint;
import com.millburnrobotics.lib.util.MathUtils;

import java.util.ArrayList;

public class PurePursuitTest {
    public static void main(String[] args) throws InterruptedException {
        double kv = 0.013069853913931;
        double ka = 0;
        double ks = 0.11659998299699;

        double claw_to_front = 3.5;
        double claw_extend = 0.5;

        double X_BLUE_BLOCK_CLAW = -24-9-claw_extend-4;
        double Y_BLUE_BLOCK_CLAW = -24-4+(9-claw_to_front);

        double X_BLUE_DELIVERY = -24-9-claw_extend;
        double Y_BLUE_DELIVERY = 72-4-34.5/2.0+(9-claw_to_front);

        double y2 = Y_BLUE_BLOCK_CLAW-8;

        ArrayList<Waypoint> w3 = new ArrayList<>(); // cycle 2
        w3.add(new Waypoint(new Pose(X_BLUE_DELIVERY, Y_BLUE_DELIVERY+2,Math.PI),Math.toRadians(150)));
        w3.add(new Waypoint(new Pose(-40,0,Math.PI),Math.PI));
        w3.add(new Waypoint(new Pose(X_BLUE_BLOCK_CLAW,y2,Math.PI),Math.toRadians(215)));
        Path path =  PathBuilder.buildPath(w3);

        Pose p;
        double t = 0, x = 0;
        double inc = 0.05;

        graphField();
        do {
            p = path.get(x);
            path.update(p);
            Pose next = path.nextPose(p);
            double basepower = path.getMotionState().v*kv+path.getMotionState().a*ka;
            double power = basepower+MathUtils.sgn(basepower)*ks;

            System.out.println("(x-" + p.x + ")^{2}+(y-" + p.y + ")^{2}=0.2");
            System.out.println("(x-" + (next.x) + ")^{2}+(y-" + next.y + ")^{2}=0.05");
            System.out.println("(" + x/100.0 + "," + power + ")");

            t += inc;
            x = path.getProfile().get(t).x;
            Thread.sleep(10);
        } while (!p.equals(path.end()));
    }
    public static void graphField() {
        System.out.println("x=-24\\left\\{-72<y<-24\\right\\}");
        System.out.println("x=-20\\left\\{-72<y<-24\\right\\}");
        System.out.println("y=-72\\left\\{-24<x<-20\\right\\}");
        System.out.println("y=-64\\left\\{-24<x<-20\\right\\}");
        System.out.println("y=-56\\left\\{-24<x<-20\\right\\}");
        System.out.println("y=-48\\left\\{-24<x<-20\\right\\}");
        System.out.println("y=-40\\left\\{-24<x<-20\\right\\}");
        System.out.println("y=-32\\left\\{-24<x<-20\\right\\}");
        System.out.println("y=-24\\left\\{-24<x<-20\\right\\}");
    }
}
