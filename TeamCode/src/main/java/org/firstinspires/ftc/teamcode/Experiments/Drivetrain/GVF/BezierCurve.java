package org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF;
import org.opencv.core.Point;
import java.util.ArrayList;

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

    public double length(Point p) {
        return Math.sqrt(p.x*p.x+p.y*p.y);
    }

    public Point derivative(double t){
        //derivative of quadratic bezier curve with respect to x
        double dx = 2*(1-t)*(p1.x-p0.x) + 2*t*(p2.x-p1.x);
        //derivative of quadratic bezier curve with respect to y
        double dy = 2*(1-t)*(p1.y-p0.y) + 2*t*(p2.y-p1.y);

        //final derivative change in x over change in y
        return new Point(dx, dy);
    }

    public Point forward(double t) {
        //set new parametric parameter
        this.T = t;

        //calculate x and y of quadratic bezier curve as a parametric
        double x = ((1 - t) * (1 - t)) * p0.x + 2 * (1 - t) * t * p1.x + (t * t) * p2.x;
        double y = ((1 - t) * (1 - t)) * p0.y + 2 * (1 - t) * t * p1.y + (t * t) * p2.y;

        return new Point(x, y);
    }

    public ArrayList<Double> arc_length_param(double dist) {
        ArrayList<Double> even_t = new ArrayList<>();
        double last_t = 0.0;
        even_t.add(0.0);
        while (last_t < 1) {
            double dDdt = this.length(this.derivative(last_t));
            last_t += dist/dDdt;
            even_t.add(last_t);
        }
        return even_t;
    }
}