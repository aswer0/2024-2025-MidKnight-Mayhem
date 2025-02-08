package org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVFNew;

import org.opencv.core.Point;

public class CompositePath {
    BezierPath[] F;
    int n_bz;
    double pid_dist;
    double end_heading;
    Point final_point;

    // Path constructor with Bezier curves
    public CompositePath(double pid_dist, double end_heading, BezierPath... F) {
        this.F = F;
        this.n_bz = F.length;
        this.pid_dist = pid_dist;
        this.end_heading = end_heading;
        this.final_point = forward(n_bz);
    }

    public int bz_index(double t) {
        int bz = (int)Math.floor(t);
        if (bz >= n_bz) bz = n_bz-1;
        if (bz < 0) bz = 0;
        return bz;
    }

    public BezierPath get_bz(double t) {
        return F[bz_index(t)];
    }

    public double local_t(double t) {
        return t-bz_index(t);
    }

    public Point forward(double t){
        int i = bz_index(t);
        return F[i].forward(t-i);
    }

    public Point derivative(double t) {
        int i = bz_index(t);
        return F[i].derivative(t-i);
    }

    public Point second_derivative(double t) {
        int i = bz_index(t);
        return F[i].second_derivative(t-i);
    }

    public double curvature(double t) {
        int i = bz_index(t);
        return F[i].curvature(t-i);
    }

    public int dDdt_sign(Point p, double t) {
        int i = bz_index(t);
        return F[i].dDdt_sign(p,t-i);
    }
}
