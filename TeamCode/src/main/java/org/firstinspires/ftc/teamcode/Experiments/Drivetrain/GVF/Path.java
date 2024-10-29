package org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF;
import org.opencv.core.Point;
import java.util.ArrayList;
import java.util.Arrays;

public class Path {
    BezierCurve[] F;
    int n_bz;
    double est_arclen;
    Point final_point;

    // Path constructor with Bezier curves
    public Path(BezierCurve[] F){
        this.F = F;
        this.n_bz = F.length;
        this.est_arclen = est_arclen();
    }

    // Path constructor with control points
    public Path(Point[][] cp){
        this.n_bz = cp.length;

        //add piecewise bezier curve
        ArrayList<BezierCurve> temp_F = new ArrayList<BezierCurve>();
        for (int i = 0; i < this.n_bz; i++){
            temp_F.add(new BezierCurve(cp[i]));
        }
        this.F = temp_F.toArray(new BezierCurve[temp_F.size()]);
        this.est_arclen = est_arclen();

        this.final_point = forward(n_bz);
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

    //calculates Bezier curve
    public Point forward(double t){
        int i = get_bz(t);
        return F[i].forward(t-i);
    }

    public Point derivative(double t) {
        int i = get_bz(t);
        return F[i].derivative(t-i);
    }

    public int dDdt_sign(Point p, double t) {
        int i = get_bz(t);
        return F[i].dDdt_sign(p,t-i);
    }
}
