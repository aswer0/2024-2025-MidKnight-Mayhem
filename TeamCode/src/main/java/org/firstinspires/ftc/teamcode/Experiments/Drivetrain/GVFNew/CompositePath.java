package org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVFNew;

import org.opencv.core.Point;

public class CompositePath {
    BezierPath[] F;
    int n_bz;
    double est_arclen;
    double pid_dist;
    double end_heading;
    Point final_point;

    // Path constructor with Bezier curves
    public CompositePath(double pid_dist, double end_heading, BezierPath... F) {
        this.F = F;
        this.n_bz = F.length;
        this.pid_dist = pid_dist;
        this.end_heading = end_heading;
        this.est_arclen = est_arclen();
        this.final_point = forward(n_bz);
    }

    public int get_bz_index(double t) {
        int bz = (int)Math.floor(t);
        if (bz >= n_bz) bz = n_bz-1;
        if (bz < 0) bz = 0;
        return bz;
    }

    public BezierPath get_bz(double t) {
        return F[get_bz_index(t)];
    }

    public double est_arclen() {
       double len = 0;
       for (int i = 0; i < n_bz; i++) {
           len += F[i].est_arclen;
       }
       return len;
    }

    public Point forward(double t){
        int i = get_bz_index(t);
        return F[i].forward(t-i);
    }

    public Point derivative(double t) {
        int i = get_bz_index(t);
        return F[i].derivative(t-i);
    }

    public Point second_derivative(double t) {
        int i = get_bz_index(t);
        return F[i].second_derivative(t-i);
    }

    public double curvature(double t) {
        int i = get_bz_index(t);
        return F[i].curvature(t-i);
    }

    public int dDdt_sign(Point p, double t) {
        int i = get_bz_index(t);
        return F[i].dDdt_sign(p,t-i);
    }
}
