package org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVFNew;
import org.opencv.core.Point;

public class BezierPath {
    // Declare control points as (x, y)
    Point[] P;

    // Degree of curve
    int K;

    // Interpolates heading
    enum HeadingMethod {
        pid,
        linear,
        path
    }

    HeadingMethod heading_method;
    public double start_heading, end_heading;
    Point final_point;

    // Estimates arc length
    int est_arclen_steps = 20;
    double[] arclen = new double[est_arclen_steps+1];
    double total_arclen;

    // Curvature power: Amount the robot slows down per unit curvature
    // For example, if you want curvature 0.05 with radius 20 inches to decrease speed by 0.3
    // you have curvature_power_decay = 0.3/0.05 = 6
    double min_power, max_power, max_turn_power, curvature_power_decay;

    public BezierPath(double min_power,
                      double max_power,
                      double max_turn_power,
                      double curvature_power_decay,
                      Point... P) {
        // Set all control points
        this.P = P;
        this.K = P.length-1;

        // Drive speed constants
        this.min_power = min_power;
        this.max_power = max_power;
        this.max_turn_power = max_turn_power;
        this.curvature_power_decay = curvature_power_decay;

        // Get estimated arc length
        this.heading_method = HeadingMethod.path;
        set_arclen();

        // Save final point
        this.final_point = forward(1);
    }

    public BezierPath pid_heading(double end_heading) {
        this.heading_method = HeadingMethod.pid;
        this.end_heading = end_heading;
        return this;
    }

    public BezierPath linear_heading(double start_heading, double end_heading) {
        this.heading_method = HeadingMethod.linear;
        this.start_heading = start_heading;
        this.end_heading = end_heading;
        return this;
    }

    public void set_arclen() {
        double cur_t = 0;
        double step = 1.0/est_arclen_steps;
        arclen[0] = 0;
        for (int i = 0; i < est_arclen_steps; i++) {
            arclen[i+1] = arclen[i]+Utils.dist(forward(cur_t+step), forward(cur_t));
            cur_t += step;
        }
        total_arclen = arclen[est_arclen_steps];
    }

    public double extrapolate_len(int section) {
        return (arclen[section+1]-arclen[section])*est_arclen_steps;
    }

    public double get_arclen(double t) {
        if (t >= 0 && t <= est_arclen_steps) {
            int nearest = (int)(t*est_arclen_steps);
            double part_section = t*est_arclen_steps - nearest;
            return (1-part_section)*arclen[nearest]+part_section*arclen[nearest+1];
        } else if (t < 0) {
            return t*extrapolate_len(0);
        } else {
            return total_arclen+(t-1)*extrapolate_len(est_arclen_steps-1);
        }
    }

    public double dist_to_t(double dist) {
        if (dist >= 0 && dist < arclen[est_arclen_steps]) {
            int l_bound = 0, r_bound = est_arclen_steps, mid;
            while (r_bound > l_bound+1) {
                mid = (l_bound+r_bound)/2;
                if (dist >= arclen[mid]) {
                    l_bound = mid;
                } else {
                    r_bound = mid;
                }
            }
            double part_section = (dist-arclen[l_bound])/(arclen[r_bound]-arclen[l_bound]);
            return (l_bound+part_section)/est_arclen_steps;
        } else if (dist < 0) {
            return dist/extrapolate_len(0);
        } else {
            return 1+(dist-total_arclen)/extrapolate_len(est_arclen_steps-1);
        }
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