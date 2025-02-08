package org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVFNew;
import org.opencv.core.Point;

public class BezierPath {
    // Declare control points as (x, y)
    Point[] P;

    // Degree of curve
    int K;

    // Interpolates heading
    boolean interpolate_heading;
    public double target_heading;

    // Estimates arc length
    double est_arclen;

    // Speeds
    double min_power, max_power, max_turn_power;

    public BezierPath(double min_power, double max_power, double max_turn_power, Point... P) {
        // Set all control points
        this.P = P;
        this.K = P.length-1;

        // Min speed and max speed
        this.min_power = min_power;
        this.max_power = max_power;
        this.max_turn_power = max_turn_power;

        // Get estimated arc length
        this.interpolate_heading = false;
        this.est_arclen = est_arclen(0.05);
    }

    public BezierPath set_heading(double end_heading) {
        this.interpolate_heading = true;
        this.target_heading = end_heading;
        return this;
    }

    public double est_arclen(double step) {
        double len = 0;
        for (double i = 0; i < 1; i += step) {
            len += Utils.len_v(Utils.sub_v(this.forward(i+step), this.forward(i)));
        }
        return len;
    }

    public Point forward(double t) {
        //calculate x and y of the bezier curve as a parametric
        double x = 0.0;
        double y = 0.0;
        int cur_comb = 1;
        double coeff;
        for (int i = 0; i <= K; i++) {
            coeff = cur_comb*Math.pow(t, i)*Math.pow(1-t, K-i);
            x += coeff*P[i].x;
            y += coeff*P[i].y;
            cur_comb *= (K-i);
            cur_comb /= i+1;
        }
        return new Point(x, y);
    }

    public Point derivative(double t){
        //calculate x and y of the bezier curve as a parametric
        double dx = 0.0;
        double dy = 0.0;
        int cur_comb = K;
        double coeff;
        int new_k = K-1;
        for (int i = 0; i <= new_k; i++) {
            coeff = cur_comb*Math.pow(t, i)*Math.pow(1-t, new_k-i);
            dx += coeff*(P[i+1].x-P[i].x);
            dy += coeff*(P[i+1].y-P[i].y);
            cur_comb *= (new_k-i);
            cur_comb /= i+1;
        }
        return new Point(dx, dy);
    }

    public Point second_derivative(double t){
        //calculate x and y of the bezier curve as a parametric
        double dx = 0.0;
        double dy = 0.0;
        int cur_comb = K*(K-1);
        double coeff;
        int new_k = K-2;
        for (int i = 0; i <= new_k; i++) {
            coeff = cur_comb*Math.pow(t, i)*Math.pow(1-t, new_k-i);
            dx += coeff*(P[i+2].x-2*P[i+1].x+P[i].x);
            dy += coeff*(P[i+2].y-2*P[i+1].y+P[i].y);
            cur_comb *= (new_k-i);
            cur_comb /= i+1;
        }
        return new Point(dx, dy);
    }

    public double curvature(double t) {
        double d_slope = Utils.slope_v(derivative(t));
        double second_d_slope = Utils.slope_v(second_derivative(t));
        return second_d_slope/Math.pow(1+d_slope*d_slope, 1.5);
    }

    // Sign of the derivative of the distance from point to curve
    // Useful for finding the closest point on the curve to a given position
    public int dDdt_sign(Point p, double t) {
        return (int)Math.signum(Utils.sub_v(forward(t), p).dot(derivative(t)));
    }
}
