package org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF;
import org.opencv.core.Point;
import java.util.ArrayList;
import java.util.Arrays;

public class Path {
    BezierCurve[] F;
    double closest_T;
    int n_bz;

    // Path constructor
    public Path(Point[][] cp){
        this.closest_T = 0.0;
        this.n_bz = cp.length;
        ArrayList<BezierCurve> temp_F = new ArrayList<>();
        
        //add piecewise bezier curve
        for (int i = 0; i < this.n_bz; i++){
            temp_F.add(new BezierCurve(cp[i]));
        }
        this.F = (BezierCurve[]) temp_F.toArray();
    }             
  
    //calculates Bezier curve
    public Point forward(double t){
        int i = (int)Math.floor(t);
        return F[i].forward(t-i);
    }

    public Point derivative(double t){
        int i = (int)Math.floor(t);
        return F[i].derivative(t-i);
    }

    public void update_closest(Point p) {
        // Gets the closest point on the current curve to p
        int i = (int)Math.floor(closest_T);
        F[i].update_closest(p);
        closest_T = i+F[i].closest_T;

        // Updates the new curve closest_T might be on
        i = (int)Math.floor(closest_T);
        if (i < 0) i = 0;
        if (i >= this.n_bz) i = n_bz-1;
        F[i].closest_T = closest_T-i;
    }

    public Point get_v(Point p, double speed) {
        // Updates the closest point on the curve
        update_closest(p);

        // Returns the final vector
        int i = (int)Math.floor(closest_T);
        return F[i].get_v(p, speed);
    }

    public ArrayList<Double> arc_length_param(double d){
        int i = (int)Math.floor(d);
        return F[i].arc_length_param(d);
    }
}
