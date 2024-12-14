package org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVFSimplfied;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDController;
import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDFController;
import org.opencv.core.Point;

import java.util.ArrayList;

@Config
public class Path {
    public static double xp = 0.04, xi = 0, xd = 0.001;
    public static double yp = 0.04, yi = 0, yd = 0.001;
    public static double hp = 0.0065, hi = 0, hd = 0.00004;

    public static double follow_xp = 0.035, follow_xi = 0, follow_xd = 0.001;
    public static double follow_yp = 0.035, follow_yi = 0, follow_yd = 0.001;
    public static double follow_hp = 0.0095, follow_hi = 0, follow_hd = 0.00004;

    WheelControl wheelControl;
    Odometry odometry;
    Telemetry telemetry;
    BezierCurve bz;

    double D;
    double speed;
    double threshold;
    double end_angle;
    double power;
    double conv_point;
    Double[] even_t;

    PIDController heading;
    PIDController x_pos;
    PIDController y_pos;

    PIDController follow_heading;
    PIDController follow_x_pos;
    PIDController follow_y_pos;

    public Path(Point[] cp, WheelControl w, Odometry odometry, Telemetry telemetry, 
                double speed, 
                double threshold,
                double end_angle,
                double power)
    {
        this.wheelControl = w;
        this.odometry = odometry;
        this.telemetry = telemetry;
        this.bz = new BezierCurve(cp);

        this.speed = speed;
        this.threshold = threshold;
        this.end_angle = end_angle;
        this.power = power;
        this.D = 0.0;
        this.conv_point = 0.78;

        //ArrayList<Double> temp_even_t = bz.arc_length_param(speed);
        //even_t = temp_even_t.toArray(new Double[temp_even_t.size()]);

        heading = new PIDController(hp, hi, hd);
        x_pos = new PIDController(xp, xi, xd);
        y_pos = new PIDController(yp, yi, yd);

        follow_heading = new PIDController(follow_hp, follow_hi, follow_hd);
        follow_x_pos = new PIDController(follow_xp, follow_xi, follow_xd);
        follow_y_pos = new PIDController(follow_yp, follow_yi, follow_yd);
    }

    public void update(boolean use_end_angle){
        Point d = this.bz.derivative(this.D);
        Point p = this.bz.forward(this.D);
        double target_angle;

        if (this.D >= this.conv_point && use_end_angle){
            target_angle = converge(this.D, this.end_angle, 0);
        }
        else{
            target_angle = Math.toDegrees(Math.atan2(d.y, d.x));
        }

        double dist = this.get_dist(p, new Point(odometry.opt.get_x(), odometry.opt.get_y()));

        this.pid_to_point(p, target_angle);

        if (0.0 <= this.D && this.D <= 1) {
            if (dist < threshold) {
                this.D += this.speed;
            }
        }

    }

    public void retrace(boolean use_end_angle) {
        Point d = this.bz.derivative(this.D);
        Point p = this.bz.forward(this.D);
        double target_angle;

        if (this.D <= 1-this.conv_point && use_end_angle){
            target_angle = converge(this.D, this.end_angle, 0);
        } else {
            target_angle = Math.toDegrees(Math.atan2(d.y, d.x));
        }

        double dist = this.get_dist(p, new Point(odometry.opt.get_x(), odometry.opt.get_y()));

        this.pid_to_point(p, target_angle);

        if (0.0 <= this.D && this.D <= 1) {
            if (dist < threshold) {
                this.D -= this.speed;
            }
        }
    }

    public void update(double target_angle){
        Point p = this.bz.forward(this.D);
        double dist = this.get_dist(p, new Point(odometry.opt.get_x(), odometry.opt.get_y()));

        target_angle = converge(this.D, target_angle, 0);
        this.pid_to_point(p, target_angle);

        if (0.0 <= this.D && this.D <= 1) {
            if (dist < threshold) {
                this.D += this.speed;
            }
        }

    }

    public void pid_to_point(Point p, double target_angle){
        double x_error = x_pos.calculate(this.odometry.opt.get_x(), p.x);
        double y_error = y_pos.calculate(this.odometry.opt.get_y(), p.y);
        double head_error = heading.calculate(this.odometry.opt.get_heading(), target_angle);

        if (Math.abs(target_angle-odometry.opt.get_heading()) <= 1){
            head_error = 0;
        }

        wheelControl.drive(-x_error, -y_error, head_error, Math.toRadians(odometry.opt.get_heading()), this.power);
    }

    public void follow_pid_to_point(Point p, double target_angle){
        double x_error = follow_x_pos.calculate(this.odometry.opt.get_x(), p.x);
        double y_error = follow_y_pos.calculate(this.odometry.opt.get_y(), p.y);
        double head_error = follow_heading.calculate(this.odometry.opt.get_heading(), target_angle);

        if (Math.abs(target_angle-odometry.opt.get_heading()) <= 1){
            head_error = 0;
        }

        wheelControl.drive(-x_error, -y_error, head_error, Math.toRadians(odometry.opt.get_heading()), this.power);
    }

    public void set_new_path(Point[] cp){
        this.bz = new BezierCurve(cp);
    }
    public void set_end_angle(double end_angle){
        this.end_angle = end_angle;
    }
    public void set_d(double d){
        this.D = d;
    }

    public double get_d_at_t(){
        int index = (int) (this.D * (this.even_t.length - 1));
        return this.even_t[index];
    }

    public double get_d(){
        return this.D;
    }
    
    public double get_dist(Point p1, Point p2){
        return Math.sqrt((p2.x-p1.x)*(p2.x-p1.x) + (p2.y-p1.y)*(p2.y-p1.y));
    }
    public boolean at_point(Point p, double collision_dist){
        return get_dist(
                new Point(odometry.opt.get_x(), odometry.opt.get_y()),
                p
        ) <= collision_dist;
    }
    public boolean at_point(double collision_dist){
        return get_dist(
                new Point(odometry.opt.get_x(), odometry.opt.get_y()),
                new Point(this.bz.P[this.bz.K].x, this.bz.P[this.bz.K].y)
        ) <= collision_dist;
    }
    public double converge(double t, double v_max, double v_min){
        double A = v_min-v_max;
        return A * (t-1) * (t-1) + v_max;
    }

    public void stop(){
        this.wheelControl.drive(0, 0, 0, 0, 0);
    }
}
