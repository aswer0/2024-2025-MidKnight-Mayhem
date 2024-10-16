package org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVFSimplfied;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDController;
import org.opencv.core.Point;

import java.util.ArrayList;

public class Path {
    WheelControl wheelControl;
    Odometry odometry;
    BezierCurve bz;

    double D;
    double speed;
    Double[] even_t;

    PIDController heading;
    PIDController x_pos;
    PIDController y_pos;

    public Path(Point[] cp, WheelControl w, Odometry odometry, double speed, double init_p){
        this.wheelControl = w;
        this.odometry = odometry;
        this.bz = new BezierCurve(cp);

        this.speed = speed;
        this.D = init_p;

        //ArrayList<Double> temp_even_t = bz.arc_length_param(speed);
        //even_t = temp_even_t.toArray(new Double[temp_even_t.size()]);

        heading = new PIDController(1.0, 0.0, 1.0);
        x_pos = new PIDController(1.0, 0.0, 1.0);
        y_pos = new PIDController(1.0, 0.0, 1.0);
    }

    public double follow_path(){
        Point d = this.bz.derivative(this.D);
        Point t = this.bz.forward(this.D);

        double dx = d.x;
        double dy = d.y;

        double target_angle = Math.toDegrees(Math.atan2(dy, dx));
        if (target_angle < 0) {
            target_angle += 360;
        }

        double x_error = x_pos.calculate(this.odometry.opt.get_x(), t.y);
        double y_error = y_pos.calculate(this.odometry.opt.get_y(), t.x);
        double head_error = heading.calculate(this.odometry.opt.get_heading() ,target_angle);

        wheelControl.drive(y_error, x_error, head_error, 0, 0.4);
        this.D += this.speed;

        return target_angle;
    }

    public double get_d_at_t(){
        int index = (int) (this.D * (this.even_t.length - 1));
        return this.even_t[index];
    }

    public double get_d(){
        return this.D;
    }

    public void stop(){
        this.wheelControl.drive(0, 0, 0, 0, 0);
    }
}
