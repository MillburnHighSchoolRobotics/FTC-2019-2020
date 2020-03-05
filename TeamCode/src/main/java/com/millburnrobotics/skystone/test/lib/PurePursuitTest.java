package com.millburnrobotics.skystone.test.lib;

import com.millburnrobotics.lib.control.Path;
import com.millburnrobotics.lib.control.PathBuilder;
import com.millburnrobotics.lib.geometry.Pose;
import com.millburnrobotics.lib.geometry.Waypoint;
import com.millburnrobotics.lib.profile.MotionState;
import com.millburnrobotics.lib.util.MathUtils;

import java.util.ArrayList;

import static com.millburnrobotics.skystone.Constants.DriveConstants.MAX_V;

public class PurePursuitTest {
    public static void main(String[] args) throws InterruptedException {
        double kv = 0.01439;
        double ka = 0.000146;
        double ks = 0.04712;;

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

        Pose p = new Pose(-24-9-0.5, 72-4-34.5/2.0+(9-3.5)+2,Math.PI);
        double v = path.length()/(path.duration());
        int x = 0;
        graphField();
        while (!p.equals(path.end())) {
            path.update(p);
            Pose pose1 = path.nextPose(p);
            System.out.println("(x-" + p.x + ")^{2}+(y-" + p.y + ")^{2}=0.2");
            System.out.println("(x-" + (pose1.x) + ")^{2}+(y-" + pose1.y + ")^{2}=0.05");

            double basepower = path.getMotionState().v*kv+path.getMotionState().a*ka;
            double power = basepower+MathUtils.sgn(basepower)*ks;
            System.out.println("(" + (x/150.0) + "," + power + ")");
//            System.out.println(pose1.distTo(p));
            if (pose1.equals(path.end())) {
                break;
            }
            double dx = pose1.x-p.x;
            double dy = pose1.y-p.y;
            double alpha = Math.atan2(dy,dx);
            double dx1 = v*Math.cos(alpha)/100.0;
            double dy1 = v*Math.sin(alpha)/100.0;

            p = new Pose(p.x+dx1,p.y+dy1);
            x++;
        }
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
