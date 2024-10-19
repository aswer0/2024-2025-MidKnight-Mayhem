package org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVFSimplfied;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDController;
import org.opencv.core.Point;

import java.util.ArrayList;

public class Path {
    WheelControl wheelControl;
    Odometry odometry;
    Telemetry telemetry;
    BezierCurve bz;

    double D;
    double increment;
    Double[] even_t;

    PIDController heading;
    PIDController x_pos;
    PIDController y_pos;

    public Path(Point[] cp, WheelControl w, Odometry odometry, Telemetry telemetry, double increment){
        this.wheelControl = w;
        this.odometry = odometry;
        this.telemetry = telemetry;
        this.bz = new BezierCurve(cp);

        this.increment = increment;
        this.D = 0.0;

        //ArrayList<Double> temp_even_t = bz.arc_length_param(increment);
        //even_t = temp_even_t.toArray(new Double[temp_even_t.size()]);

        heading = new PIDController(0.05, 0.0, 0.0001);
        x_pos = new PIDController(0.022, 0.0, 0);
        y_pos = new PIDController(0.022, 0.0, 0);
    }

    public double update(double power){
        Point d = this.bz.derivative(this.D);
        Point p = this.bz.forward(this.D);
        double target_angle = Math.toDegrees(Math.atan2(d.y, d.x));

        this.pid_to_point(p, target_angle);

        if (0.0 <= this.D && this.D <= 1.0) {
            this.D += this.increment;
        }

        return target_angle;
    }
    public double pid_to_point(Point p, double target_angle){
        double x_error = x_pos.calculate(this.odometry.opt.get_x(), p.x);
        double y_error = y_pos.calculate(this.odometry.opt.get_y(), p.y);
        double head_error = heading.calculate(this.odometry.opt.get_heading(), target_angle);

        telemetry.addData("Error x", x_error);
        telemetry.addData("Error y", y_error);
        telemetry.addData("Error head", head_error);

        wheelControl.drive(x_error, -y_error, -head_error, -Math.toRadians(odometry.opt.get_heading()), 0.4);

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
