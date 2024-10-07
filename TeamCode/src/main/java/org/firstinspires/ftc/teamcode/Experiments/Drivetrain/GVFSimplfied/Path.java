package org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVFSimplfied;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.opencv.core.Point;

import java.util.ArrayList;

public class Path {
    WheelControl wheelControl;
    BezierCurve bz;

    double D;
    double speed;
    Double[] even_t;

    public Path(Point[] cp, WheelControl w, double speed){
        this.wheelControl = w;
        this.bz = new BezierCurve(cp[0], cp[1], cp[2], cp[3]);

        this.speed = speed;
        this.D = 0.0;

        ArrayList<Double> temp_even_t = bz.arc_length_param(speed);
        even_t = temp_even_t.toArray(new Double[temp_even_t.size()]);

    }

    public void follow_path(double target_angle){
        Point d = this.bz.derivative(this.get_d_at_t());
        double x = d.x;
        double y = d.y;

        wheelControl.drive(y, x, 2.5, target_angle, 0.25);
        this.D += this.speed;
    }

    public double get_d_at_t(){
        int index = (int) (this.D * (this.even_t.length - 1));
        return this.even_t[index];
    }


}
