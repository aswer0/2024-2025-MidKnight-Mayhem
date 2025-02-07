package org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVFNew;

import org.opencv.core.Point;

import java.util.ArrayList;

public class CompositePath {
    BezierPath[] F;
    int n_bz;
    double est_arclen;
    double pid_dist;
    Point final_point;

    // Path constructor with Bezier curves
    public CompositePath(double pid_dist, BezierPath[] F) {
        this.F = F;
        this.n_bz = F.length;
        this.pid_dist = pid_dist;
        this.est_arclen = est_arclen();
    }

    public int get_bz(double t) {
        int bz = (int)Math.floor(t);
        if (bz >= n_bz) bz = n_bz-1;
        if (bz < 0) bz = 0;
        return bz;
    }

    public double est_arclen() {
       double len = 0;
       for (int i = 0; i < n_bz; i++) {
           len += F[i].est_arclen;
       }
       return len;
    }
}
