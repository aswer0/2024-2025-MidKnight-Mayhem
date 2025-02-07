package org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVFNew;
import org.opencv.core.Point;

// Point utils
public class Utils {
    public static Point add_v(Point... p) {
        double x = 0;
        double y = 0;
        for (Point i:p) {
            x += i.x; y += i.y;
        }
        return new Point(x, y);
    }

    public static Point sub_v(Point a, Point b) {
        return new Point(a.x-b.x, a.y-b.y);
    }

    public static Point mul_v(Point... p) {
        double x = 1;
        double y = 1;
        for (Point i:p) {
            x *= i.x; y *= i.y;
        }
        return new Point(x, y);
    }

    public static Point mul_v(Point a, double b) {
        return new Point(a.x*b, a.y*b);
    }

    public static Point div_v(Point a, Point b) {
        return new Point(a.x/b.x, a.y/b.y);
    }

    public static Point div_v(Point a, double b) {
        return new Point(a.x/b, a.y/b);
    }

    // Length of vector
    public static double len_v(Point v) {
        return Math.sqrt(v.x*v.x+v.y*v.y);
    }

    // Distance between two points
    public static double dist(Point a, Point b) {
        return Utils.len_v(Utils.sub_v(a, b));
    }

    // Scale vector to length
    public static Point scale_v(Point v, double new_len) {
        double scale_factor = new_len/ len_v(v);
        return new Point(v.x*scale_factor, v.y*scale_factor);
    }

    // Angle of vector
    public static double angle_v(Point v) {
        double angle = Math.atan2(v.y, v.x);
        if (angle < 0) angle += Math.PI * 2;
        return angle;
    }

    // Slope of vector
    public static double slope_v(Point v) {
        return v.y/v.x;
    }

    // Generates vector with slope and length
    public static Point v_with_slope(double l, double slope) {
        double denom = Math.sqrt(1+slope*slope);
        return new Point(l/denom, l*slope/denom);
    }

    // Converts polar to rectangular
    public static Point polar_to_rect(double l, double theta) {
        return new Point(l*Math.cos(theta), l*Math.sin(theta));
    }
}
