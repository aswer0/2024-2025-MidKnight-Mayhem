package org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF;
import org.opencv.core.Point;

public class BezierCurve {
    //Declare Control Points p0, p1, p2 as x and y pair
    Point p0;
    Point p1;
    Point p2;

    //Declare parametric parameter t
    double T;

    public BezierCurve(Point p0, Point p1, Point p2){
        // initialize parametric parameter as 0
        this.T = 0.0;

        // set all control points
        this.p0 = p0;
        this.p1 = p1;
        this.p2 = p2;

    }

    public double derivative(double t){
        //derivative of quadratic bezier curve with respect to x
        double dx = 2*(1-t)*(p1.x-p0.x) + 2*t*(p2.x-p1.x);
        //derivative of quadratic bezier curve with respect to y
        double dy = 2*(1-t)*(p1.y-p0.y) + 2*t*(p2.y-p1.y);

        //final derivative change in x over change in y
        return dy/dx;
    }

    public Point forward(double t){
        //set new parametric parameter
        this.T = t;

        //calculate x and y of quadratic bezier curve as a parametric
        double x = ((1-t)*(1-t))*p0.x + 2*(1-t)*t*p1.x + (t*t)*p2.x;
        double y = ((1-t)*(1-t))*p0.y + 2*(1-t)*t*p1.y + (t*t)*p2.y;

        return new Point(x, y);
    }

    public double arc_length_param(double d){
        return 0.0;
    }
}