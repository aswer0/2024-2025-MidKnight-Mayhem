package org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.ExperimentalDrive;
import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDController;
import org.opencv.core.Point;

public class VectorField {
    // Robot controls
    public Odometry odometry;
    ExperimentalDrive drive;
    Path path;

    // Robot tuning
    double max_speed = 0.7;
    double min_speed = 0.5;
    double max_turn_speed = 20;
    double angle_to_power = 20;
    double corr_weight = 0.1;

    // End decel: speed decrease per distance
    double end_decel = 0.02;
    double end_heading;

    // Backend variables
    public double D;
    Point velocity = new Point(0, 0);
    public double speed;
    public double turn_speed;
    public boolean PID = false;

    // PID at end of path
    double xp = 0.022, xi = 0, xd = 0;
    double yp = 0.022, yi = 0, yd = 0;
    double hp = 0.002, hi = 0, hd = 0.0001;
    PIDController x_PID;
    PIDController y_PID;
    PIDController heading_PID;

    // Constructor
    public VectorField(ExperimentalDrive w,
                       Odometry o,
                       Path p,
                       double end_heading) {
        // Inputs
        this.odometry = o;
        this.path = p;
        this.drive = w;
        this.end_heading = end_heading;

        // Zero power behavior: brake
        drive.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // PID variables at end of path
        x_PID = new PIDController(xp, xi, xd);
        y_PID = new PIDController(yp, yi, yd);
        heading_PID = new PIDController(hp, hi, hd);
    }

    // Gets closest point on path to robot
    public Point get_closest() {
        return path.forward(D);
    }

    // x position of robot
    public double get_x() {
        return odometry.opt.get_x();
    }

    // y position of robot
    public double get_y() {
        return odometry.opt.get_y();
    }

    // Heading of robot
    public double get_heading() {
        return odometry.opt.get_heading();
    }

    // Gets position of robot
    public Point get_pos() {
        return new Point(get_x(), get_y());
    }

    // Updates closest point on curve using binary search
    public void update_closest(double look_ahead,
                               double max_rough_iters,
                               double tune_iters,
                               double rate) {
        Point pos = Utils.add_v(get_pos(), Utils.mul_v(velocity, look_ahead));
        double path_len = path.F[path.get_bz(D)].est_arclen;
        double update = rate*speed/path_len;

        // Get rough estimate
        int init_sign = path.dDdt_sign(pos, D);
        int iters = 0;
        while (path.dDdt_sign(pos, D) == init_sign && iters++ < max_rough_iters) {
            D -= init_sign*update;
        }

        // Binary search to tune closest
        for (int i = 0; i < tune_iters; i++) {
            if (path.dDdt_sign(pos, D) > 0) D -= update;
            else D += update;
            update /= 2;
        }
        if (D > path.n_bz) D = path.n_bz;
    }

    // Get turn angle
    public double turn_angle(double current, double target) {
        double turn_angle = target-current;
        if (turn_angle < -180) turn_angle += 360;
        if (turn_angle > 180) turn_angle -= 360;
        return turn_angle;
    }


    // Robot's angle to path
    public double angle_to_path() {
        update_closest(0, 50, 5, 1);
        Point orth = Utils.sub_v(get_closest(), get_pos());
        orth = Utils.scale_v(orth, corr_weight*Utils.length(orth));
        Point tangent = Utils.scale_v(path.derivative(D), 1);
        return Utils.angle_v(Utils.add_v(orth, tangent));
    }

    // PID to a point given coordinates and heading
    public void pid_to_point(Point p, double target_angle, double power) {
        double x_error = p.x-get_x();
        double y_error = p.y-get_y();
        double head_error = heading_PID.calculate(get_heading(), target_angle)/power;
        drive.drive(x_error, -y_error, -head_error, -Math.toRadians(get_heading()), power);
    }

    // Move with GVF and PID at the end
    public void move() {
        // Get speed with curves and end decel
        double drive_speed = min_speed+(turn_speed/max_turn_speed)*(max_speed-min_speed);
        double end_speed = end_decel*Utils.dist(get_pos(), path.final_point);
        speed = Math.max(Math.min(drive_speed, end_speed), 0.2);

        // PID when you get close enough
        if (Utils.dist(get_pos(), path.final_point) < 5) {
            PID = true;
            pid_to_point(path.final_point, end_heading, speed); return;
        }
        PID = false;
        // Turning
        double target_angle = angle_to_path();
        turn_speed = turn_angle(get_heading(), Math.toDegrees(target_angle))/angle_to_power;
        if (turn_speed > max_turn_speed) turn_speed = max_turn_speed;
        if (turn_speed < -max_turn_speed) turn_speed = -max_turn_speed;

        // Drive according to calculations
        velocity = Utils.scale_v(new Point(Math.cos(target_angle), Math.sin(target_angle)), speed);
        drive.drive(1, 0, -turn_speed, 0, speed);
    }
}