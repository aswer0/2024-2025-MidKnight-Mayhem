package org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF;
import org.opencv.core.Point;

import java.util.ArrayList;

public class Path {
    ArrayList<BezierCurve> F;
    double T;

    public Path(ArrayList<ArrayList<Point>> cp){
        this.T = 0.0;

        for (int i=0; i<cp.size(); i+=3){
            BezierCurve bz = new BezierCurve(cp.get(i).get(0), cp.get(i).get(1), cp.get(i).get(2));
            F.add(bz);
        }
    }

    public double derivative(double t){
        int i = (int)Math.floor(t);
        Point p0 = F.get(i).p0;
        Point p1 = F.get(i).p1;
        Point p2 = F.get(i).p2;
        t -= i;

        //derivative of quadratic bezier curve with respect to x
        double dx = 2*(1-t)*(p1.x-p0.x) + 2*t*(p2.x-p1.x);
        //derivative of quadratic bezier curve with respect to y
        double dy = 2*(1-t)*(p1.y-p0.y) + 2*t*(p2.y-p1.y);

        //final derivative change in x over change in y
        return dy/dx;
    }

    public Point forward(double t){
        int i = (int)Math.floor(t);
        Point p0 = F.get(i).p0;
        Point p1 = F.get(i).p1;
        Point p2 = F.get(i).p2;

        //set new parametric parameter
        this.T = t;

        //calculate x and y of quadratic bezier curve as a parametric
        double x = ((1-t-i)*(1-t-i))*p0.x + 2*(1-t-i)*(t-i)*p1.x + ((t-i)*(t-i))*p2.x;
        double y = ((1-t-i)*(1-t-i))*p0.y + 2*(1-t-i)*(t-i)*p1.y + ((t-i)*(t-i))*p2.y;

        return new Point(x, y);
    }

    public double arc_length_param(double d){
        return 0.0;
    }
}
