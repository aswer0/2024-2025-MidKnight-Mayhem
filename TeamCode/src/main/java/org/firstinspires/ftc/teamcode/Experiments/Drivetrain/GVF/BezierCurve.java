package org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF;
import org.opencv.core.Point;

public class BezierCurve {
    // Declare control points as (x, y)
    Point[] P;

    // Degree of curve
    int K;

    double est_arclen;

    public BezierCurve(Point[] P) {
        // Set all control points
        this.P = P;
        this.K = P.length-1;

        // Get estimated arc length
        this.est_arclen = this.est_arclen(0.05);
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
