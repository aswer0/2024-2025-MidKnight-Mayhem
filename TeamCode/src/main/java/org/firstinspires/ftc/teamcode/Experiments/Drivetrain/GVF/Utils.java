package org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF;
import org.opencv.core.Point;

// Point utils
public class Utils {
    public static Point add_v(Point a, Point b) {
        return new Point(a.x+b.x, a.y+b.y);
    }

    public static Point sub_v(Point a, Point b) {
        return new Point(a.x-b.x, a.y-b.y);
    }

    public static Point mul_v(Point a, Point b) {
        return new Point(a.x*b.x, a.y*b.y);
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

    public static double len_v(Point v) {
        return Math.sqrt(v.x*v.x+v.y*v.y);
    }

    public static double dist(Point a, Point b) {
        return Utils.len_v(Utils.sub_v(a, b));
    }

    public static Point scale_v(Point v, double new_len) {
        double scale_factor = new_len/ len_v(v);
        return new Point(v.x*scale_factor, v.y*scale_factor);
    }

    public static double angle_v(Point v) {
        double angle = Math.atan2(v.y, v.x);
        if (angle < 0) angle += Math.PI * 2;
        return angle;
    }
}
