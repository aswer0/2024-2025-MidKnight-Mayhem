package org.firstinspires.ftc.teamcode.Experiments.Drivetrain;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDController;
import org.opencv.core.Point;

public class PIDdrive {
    public static double xp = 0.045, xi = 0, xd = 0.001;
    public static double yp = 0.055, yi = 0, yd = 0.001;
    public static double hp = 0.0095, hi = 0, hd = 0.00004;

    WheelControl wheelControl;
    Odometry odometry;
    HardwareMap hardwareMap;

    boolean continue_follow = true;

    PIDController heading;
    PIDController x_pos;
    PIDController y_pos;

    Point[] path;
    int N;

    public PIDdrive(Point[] path, HardwareMap hardwareMap, WheelControl wheelControl, Odometry odometry){
        this.wheelControl = wheelControl;
        this.hardwareMap = hardwareMap;
        this.odometry = odometry;

        this.path = path;
        this.N = path.length;

        heading = new PIDController(hp, hi, hd);
        x_pos = new PIDController(xp, xi, xd);
        y_pos = new PIDController(yp, yi, yd);
    }

    public void update(double power, double collision_dist){
        for (int i=0; i<this.N;){
            double dx = this.path[i].x-this.odometry.opt.get_x();
            double dy = this.path[i].y-this.odometry.opt.get_y();

            double target_angle = Math.toDegrees(Math.atan2(dy, dx));

            if (!this.at_point(this.path[i], collision_dist) && this.continue_follow){
                this.pid_to_point(this.path[i], target_angle, power);
            }
            else{
                i++;
            }
        }
    }

    public void pid_to_point(Point p, double target_angle, double power){
        double x_error = x_pos.calculate(this.odometry.opt.get_x(), p.x);
        double y_error = y_pos.calculate(this.odometry.opt.get_y(), p.y);
        double head_error = heading.calculate(this.odometry.opt.get_heading(), target_angle);

        if (Math.abs(target_angle-odometry.opt.get_heading()) <= 1){
            head_error = 0;
        }

        wheelControl.drive(x_error, -y_error, -head_error, -Math.toRadians(odometry.opt.get_heading()), power);
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

}
