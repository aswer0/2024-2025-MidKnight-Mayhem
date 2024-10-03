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

    //Declare current closest point on Bezier curve
    double closest_T;
    double epochs = 5;
    double rate = 0.1;

    // Weighs the tangent vector
    // The distance at which the robot corrects at 45 degrees
    double tangent_weight = 2;

    public BezierCurve(Point p0, Point p1, Point p2){
        // initialize parametric parameter as 0
        this.T = 0.0;

        // set all control points
        this.p0 = p0;
        this.p1 = p1;
        this.p2 = p2;
    }

    // Basic math operations for Points
    public static Point add_v(Point a, Point b) {
        return new Point(a.x+b.x, a.y+b.y);
    }

    public static Point sub_v(Point a, Point b) {
        return new Point(a.x-b.x, a.y-b.y);
    }

    public static double length(Point v) {
        return Math.sqrt(v.x*v.x+v.y*v.y);
    }

    public static Point scale_v(Point v, double len) {
        double scale_factor = len/length(v);
        return new Point(v.x*scale_factor, v.y*scale_factor);
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

    //generates evenly spaced t with specified spacing
    public ArrayList<Double> arc_length_param(double dist) {
        ArrayList<Double> even_t = new ArrayList<>();
        double last_t = 0.0;
        even_t.add(0.0);
        while (last_t < 1) {
            last_t += dist/length(derivative(last_t));
            even_t.add(last_t);
        }
        return even_t;
    }

    public double dDdt(Point p, double t) {
        Point orth_v = sub_v(forward(t), p);
        return orth_v.dot(derivative(t))/length(orth_v);
    }

    // Update the closest point using gradient descent
    public void update_closest(Point p) {
        for (int i = 0; i < epochs; i++) {
            closest_T -= rate*dDdt(p, closest_T);
        }
    }

    // Gets the vector given a robot's position
    public Point get_v(Point p, double speed) {
        update_closest(p);
        Point orth_v = sub_v(forward(closest_T), p);
        Point tangent_v = scale_v(derivative(closest_T), tangent_weight);
        return scale_v(add_v(orth_v, tangent_v), speed);
    }
}