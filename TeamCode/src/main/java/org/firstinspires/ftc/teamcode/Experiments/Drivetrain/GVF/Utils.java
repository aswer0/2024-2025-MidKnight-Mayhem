package org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF;

import org.opencv.core.Point;

// Point utils
public class Utils {
    public static double limit_angle_deg(double angle) {
        angle %= 360;
        if (angle > 180) angle -= 360;
        return angle;
    }

    public static double limit_angle_rad(double angle) {
        angle %= Math.PI*2;
        if (angle > Math.PI) angle -= Math.PI*2;
        return angle;
    }

    public static double linear_interpolate(double min, double max, double part) {
        return (1-part)*min+part*max;
    }

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
    public static double angle_v_rad(Point v) {
        double angle = Math.atan2(v.y, v.x);
        if (angle < 0) angle += Math.PI * 2;
        return angle;
    }

    public static double angle_v_deg(Point v) {
        return Math.toDegrees(angle_v_rad(v));
    }

    // Slope of vector
    public static double slope_v(Point v) {
        return v.y/v.x;
    }

    // Converts polar to rectangular
    public static Point polar_to_rect_rad(double l, double theta) {
        return new Point(l*Math.cos(theta), l*Math.sin(theta));
    }

    public static Point polar_to_rect_deg(double l, double theta) {
        return polar_to_rect_rad(l, Math.toDegrees(theta));
    }
}
