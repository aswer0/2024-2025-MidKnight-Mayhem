package org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF;
import org.opencv.core.Point;
import java.util.ArrayList;

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
            len += Utils.length(Utils.sub_v(this.forward(i+step), this.forward(i)));
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
            coeff = cur_comb*Math.pow(t, i)*Math.pow(1-t, i);
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
        int cur_comb = 1;
        double coeff;
        int new_k = K-1;
        for (int i = 0; i <= new_k; i++) {
            coeff = cur_comb*Math.pow(t, i)*Math.pow(1-t, i);
            dx += coeff*(P[i+1].x-P[i].x);
            dy += coeff*(P[i+1].y-P[i].y);
            cur_comb *= (new_k-i);
            cur_comb /= i+1;
        }
        // final derivative change in x over change in y
        return new Point(dx, dy);
    }

    // Sign of the derivative of the distance from point to curve
    public int dDdt_sign(Point p, double t) {
        return (int)Math.signum(Utils.sub_v(forward(t), p).dot(derivative(t)));
    }
}
