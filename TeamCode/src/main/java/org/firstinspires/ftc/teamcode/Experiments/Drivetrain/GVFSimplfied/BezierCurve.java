package org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVFSimplfied;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.Utils;
import org.opencv.core.Point;
import java.util.ArrayList;

public class BezierCurve {
    // Declare control points as (x, y)
    Point[] P;

    // Degree of curve
    int K;
    double T;
    double L;

    public BezierCurve(Point[] P){
        // initialize parametric parameter as 0
        this.T = 0.0;
        this.P = P;

        this.L = this.arc_length_param(1);
        this.K = P.length-1;
    }

    public Point forward(double t) {
        //calculate x and y of the bezier curve as a parametric
        double x = ((1-t)*(1-t))*P[0].x + 2*(1-t)*t*P[1].x + (t*t)*P[2].x;
        double y = ((1-t)*(1-t))*P[0].y + 2*(1-t)*t*P[1].y + (t*t)*P[2].y;

        return new Point(x, y);
    }

    public Point derivative(double t){
        //calculate x and y of the bezier curve as a parametric
        double dx = 2*(1-t)*(P[1].x-P[0].x) + 2*t*(P[2].x-P[1].x);
        double dy = 2*(1-t)*(P[1].y-P[0].y) + 2*t*(P[2].y-P[1].y);

        return new Point(dx, dy);
    }

    // Generates evenly spaced t with specified spacing
    /*public ArrayList<Double> arc_length_param(double dist) {
        ArrayList<Double> even_t = new ArrayList<Double>();
        double last_t = 0.0;
        even_t.add(0.0);
        while (last_t < 1) {
            last_t += dist/Utils.length(derivative(last_t));
            even_t.add(last_t);
        }
        return even_t;
    }*/

    public double d_to_t(double d){
        double epochs = 3;
        double learning_rate = 0.01;
        double initial = d/L;

        for (int i=0; i<epochs; i++){
            double t = arc_length_param(initial);
            Point p = derivative(initial);
            double deriv = 2*(t-d)*Utils.length(new Point(p.x, p.y));

            initial = initial-learning_rate*deriv;
        }

        return initial;

    }

    public double arc_length_param(double t) {
        int num_steps = 8;
        double total_length = 0;
        double step_size = t/num_steps;

        for (int i=0; i<num_steps+1; i++){
            double t_i = i*step_size;

            Point p = derivative(t_i);
            double dx = p.x;
            double dy = p.y;

            if (i == 0 || i == num_steps){
                total_length += Utils.length(new Point(dx, dy));
            }
            else if (i % 2 == 1){
                total_length += 4 * Utils.length(new Point(dx, dy));
            }
            else{
                total_length += 2 * Utils.length(new Point(dx, dy));
            }
        }

        total_length *= step_size / 3;
        return total_length;
    }
}
