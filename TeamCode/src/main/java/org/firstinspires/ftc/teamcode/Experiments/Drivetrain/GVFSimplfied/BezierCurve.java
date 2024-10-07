package org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVFSimplfied;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.Utils;
import org.opencv.core.Point;

import java.util.ArrayList;

public class BezierCurve {
    Point p0;
    Point p1;
    Point p2;
    Point p3;

    double T;

    public BezierCurve(Point p0, Point p1, Point p2, Point p3){
        this.p0 = p0;
        this.p1 = p1;
        this.p2 = p2;
        this.p3 = p3;

        this.T = 0.0;
    }

    public Point foward(double t){
        this.T = t;

        double x = Math.pow(1 - t, 3) * p0.x + 3 * Math.pow(1 - t, 2) * t * p1.x + 3 * (1 - t) * Math.pow(t, 2) * p2.x + Math.pow(t, 3) * p3.x;
        double y = Math.pow(1 - t, 3) * p0.y + 3 * Math.pow(1 - t, 2) * t * p1.y + 3 * (1 - t) * Math.pow(t, 2) * p2.y + Math.pow(t, 3) * p3.y;

        return new Point(x, y);
    }

    public Point derivative(double t){
        double dx = -3 * Math.pow(1 - t, 2) * p0.x + 3 * Math.pow(1 - t, 2) * p1.x - 6 * (1 - t) * t * p1.x + 3 * Math.pow(t, 2) * p2.x + 3 * Math.pow(t, 2) * p3.x;
        double dy = -3 * Math.pow(1 - t, 2) * p0.y + 3 * Math.pow(1 - t, 2) * p1.y - 6 * (1 - t) * t * p1.y + 3 * Math.pow(t, 2) * p2.y + 3 * Math.pow(t, 2) * p3.y;

        return new Point(dx, dy);
    }

    public ArrayList<Double> arc_length_param(double dist) {
        ArrayList<Double> even_t = new ArrayList<Double>();
        double last_t = 0.0;
        even_t.add(0.0);
        while (last_t < 1) {
            last_t += dist/ Utils.length(derivative(last_t));
            even_t.add(last_t);
        }
        return even_t;
    }
}
