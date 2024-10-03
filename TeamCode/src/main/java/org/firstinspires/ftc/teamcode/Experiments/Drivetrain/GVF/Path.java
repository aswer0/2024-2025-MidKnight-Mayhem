package org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF;
import org.opencv.core.Point;

import java.util.ArrayList;

public class Path {
    ArrayList<BezierCurve> F;
    double T;
    double close_T;

    public Path(ArrayList<ArrayList<Point>> cp){
        this.T = 0.0;
        this.close_T = 0.0;
        for (int i=0; i<cp.size(); i+=3){
            BezierCurve bz = new BezierCurve(cp.get(i).get(0), cp.get(i).get(1), cp.get(i).get(2));
            F.add(bz);
        }
    }

    public Point derivative(double t){
        int i = (int)Math.floor(t);
        Point p0 = F.get(i).p0;
        Point p1 = F.get(i).p1;
        Point p2 = F.get(i).p2;

        return F.get(i).derivative(t-i);
    }

    //calculates Bezier curve
    public Point forward(double t){
        int i = (int)Math.floor(t);

        //set new parametric parameter
        this.T = t;

        //calculate x and y of quadratic bezier curve as a parametric
        return F.get(i).forward(t-i);
    }

    public ArrayList<Double> arc_length_param(double d){
        int i = (int)Math.floor(d);

        return F.get(i).arc_length_param(d);
    }
}
