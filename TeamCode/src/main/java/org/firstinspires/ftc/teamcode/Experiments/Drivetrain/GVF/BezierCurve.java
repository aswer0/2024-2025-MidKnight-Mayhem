package org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF;
import org.opencv.core.Point;
import java.util.ArrayList;

public class BezierCurve {
    // Declare control points as (x, y)
    Point[] P;

    // Degree of curve
    int K;

    // Declare current closest point on Bezier curve
    double closest_T;

    // Gradient descent hyperparameters for finding closest point
    double epochs = 5;
    double rate = 0.1;

    // Weighs the tangent vector
    // How much the robot weighs correction
    double correction_weight = 5;

    public BezierCurve(Point[] P){
        // initialize parametric parameter as 0
        this.closest_T = 0.0;

        // set all control points
        this.P = P;
        this.K = P.length-1;
    }

    public Point forward(double t) {
        //calculate x and y of the bezier curve as a parametric
        double x = 0.0;
        double y = 0.0;
        int cur_comb = 1;
        double coeff;
        for (int i = 0; i <= K; i++) {
            coeff = cur_comb*Math.pow(t, i)*Math.pow(1-t, i);
            x += coeff*P[i].x;
            y += coeff*P[i].y;
            cur_comb *= K;
            cur_comb /= i+1;
        }
        return new Point(x, y);
    }

    public Point derivative(double t){
        //calculate x and y of the bezier curve as a parametric
        double dx = 0.0;
        double dy = 0.0;
        int cur_comb = 1;
        double coeff;
        for (int i = 0; i <= K-1; i++) {
            coeff = cur_comb*Math.pow(t, i)*Math.pow(1-t, i);
            dx += coeff*(P[i+1].x-P[i].x);
            dy += coeff*(P[i+1].y-P[i].y);
            cur_comb *= K;
            cur_comb /= i+1;
        }
        // final derivative change in x over change in y
        return new Point(dx, dy);
    }

    // Derivative of the distance from point to curve
    public double dDdt(Point p, double t) {
        Point orth_v = Utils.sub_v(forward(t), p);
        return orth_v.dot(derivative(t))/Utils.length(orth_v);
    }

    // Update the closest point using gradient descent
    public void update_closest(Point p) {
        for (int i = 0; i < epochs; i++) {
            closest_T -= rate*dDdt(p, closest_T);
        }
    }

    // Gets the vector given a robot's position
    public Point get_v(Point p, double speed) {
        Point orth_v = Utils.sub_v(forward(closest_T), p);
        Point tangent_v = Utils.scale_v(derivative(closest_T), 1/correction_weight);
        return Utils.scale_v(Utils.add_v(orth_v, tangent_v), speed);
    }

    // Generates evenly spaced t with specified spacing
    public ArrayList<Double> arc_length_param(double dist) {
        ArrayList<Double> even_t = new ArrayList<Double>();
        double last_t = 0.0;
        even_t.add(0.0);
        while (last_t < 1) {
            last_t += dist/Utils.length(derivative(last_t));
            even_t.add(last_t);
        }
        return even_t;
    }
}