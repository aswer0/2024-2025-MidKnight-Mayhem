package org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVFNew;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.Experiments.Utils.HPIDController;
import org.firstinspires.ftc.teamcode.Experiments.Utils.TestPID;
import org.opencv.core.Point;

public class VectorField {
    // Robot controls
    public Odometry odometry;
    public WheelControl drive;
    public CompositePath path;

    // Motion profiling
    double velocity_update_rate = 0.1;
    double p_to_v = 68;
    public Point prev_pos;
    public double speed = 0;
    public Point velocity = new Point(0, 0);

    // Correction constants
    double path_corr = 0.1;
    //double centripetal_corr = 0;
    //double centripetal_threshold = 10;
    double accel_corr = 0;

    // PID constants (at end of path)
    double end_decel = 0.05;
    Point end_target;

    // Heading controls
    double end_heading;

    // Backend variables
    public double D;
    public BezierPath cur_bz;
    public double strafe_angle;
    public double drive_power;
    public double turn_power;
    public boolean PID = false;
    public double error = 0;
    public ElapsedTime timer;

    // PID variables
    public double xp = end_decel, xi = 0.1, xd = 0.006, xithres = 2;
    public double yp = end_decel, yi = 0.1, yd = 0.006, yithres = 2;
    public double hp = 0.01, hi = 0.025, hd = 0.001, hithres = 2;

    TestPID x_PID;
    TestPID y_PID;
    HPIDController h_PID;

    public double x_error;
    public double y_error;

    // Constructor
    public VectorField(WheelControl w,
                       Odometry o) {
        // Inputs
        this.odometry = o;
        this.drive = w;

        // Zero power behavior: brake
        drive.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set PID controllers
        this.x_PID = new TestPID(xp, xi, xd, xithres);
        this.y_PID = new TestPID(yp, yi, yd, yithres);
        this.h_PID = new HPIDController(hp, hi, hd, hithres);

        // Timer
        timer = new ElapsedTime();
        prev_pos = get_pos();
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

    // Sets velocity measurement of robot
    public void set_velocity() {
        if (timer.seconds() < velocity_update_rate) return;
        velocity = Utils.div_v(Utils.sub_v(get_pos(), prev_pos), timer.seconds());
        speed = Utils.len_v(velocity);
        prev_pos = get_pos();
        timer.reset();
    }

    // Gets acceleration correction term
    /*public Point get_accel_corr(Point move_v) {
        if (accel_corr > 0) {
            Point accel = Utils.sub_v(move_v, Utils.div_v(velocity, p_to_v));
            return Utils.mul_v(accel, accel_corr);
        } else return new Point (0, 0);
    }*/

    // Updates closest point on curve using signed GD & binary search
    public void update_closest(double max_rough_iters,
                               double tune_iters,
                               double rate) {
        Point pos = get_pos();
        double path_len = path.get_bz(D).est_arclen;
        double update = rate*drive_power/path_len;

        // Get rough estimate
        int iters = 0;
        int init_sign = path.dDdt_sign(pos, D);
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

    // Calculates D (parameter) value that is approximately "dist" distance from end
    public double D_from_end(double dist) {
        return path.n_bz-dist/path.F[path.n_bz-1].est_arclen;
    }

    // Calculates distance to endpoint
    public double dist_to_end() {
        return Utils.dist(path.final_point, get_pos());
    }

    // Distance to closest point on path
    public double closest_dist() {
        return Utils.dist(path.forward(D), get_pos());
    }

    // Gets power when approaching end
    public double get_end_power(Point p) {
        return end_decel*Utils.dist(get_pos(), p);
    }

    // Calculates turn power based on target angle
    public void set_turn_power(double target_heading) {
        turn_power = h_PID.calculate(get_heading(), target_heading);
        if (turn_power > cur_bz.max_turn_power) turn_power = cur_bz.max_turn_power;
        if (turn_power < -cur_bz.max_turn_power) turn_power = -cur_bz.max_turn_power;
    }

    // Robot's move vector to path
    public double get_strafe_angle() {
        // Base vector (orthogonal & tangent)
        Point orth = Utils.mul_v(Utils.sub_v(get_closest(), get_pos()), path_corr);
        Point tangent = Utils.scale_v(path.derivative(D), 1);
        Point move_v = Utils.add_v(orth, tangent);

        // Acceleration correction
        //Point accel_corr_term = get_accel_corr(move_v);

        // Centripetal correction
        /*Point centripetal = new Point(0, 0);
        if (closest_dist() < centripetal_threshold) {
            double perp_angle = Utils.angle_v(tangent)+Math.PI/2;
            double centripetal_len = path.curvature(D)*centripetal_corr*speed;
            centripetal = Utils.polar_to_rect(centripetal_len, perp_angle);
        }*/

        // Add everything
        return Math.toDegrees(Utils.angle_v(move_v));
        //return Utils.add_v(Utils.scale_v(move_v, speed), accel_corr_term);
    }

    // Move to a point given coordinates and heading
    public void pid_to_point(Point p, double target_heading, double max_power) {
        end_target = p;

        // PID errors
        x_error = x_PID.calculate(get_x(), p.x);
        y_error = y_PID.calculate(get_y(), p.y);
        turn_power = h_PID.calculate(get_heading(), target_heading);

        // Drive
        drive.drive_limit_power(x_error, y_error, turn_power, max_power, get_heading());
    }

    public void set_drive_power(double turn_power) {
        double max_p = cur_bz.max_power, min_p = cur_bz.min_power;
        drive_power = max_p-turn_power*(max_p-min_p)/cur_bz.max_turn_power;
        drive_power = Math.min(drive_power, get_end_power(path.final_point));
    }

    // Move with GVF
    public void follow(CompositePath path) {
        if (this.path != path) {
            D = 0; this.path = path;
        }

        end_target = path.final_point;

        update_closest(50, 5, 1);
        cur_bz = path.get_bz(D);
        
        // PID at the end
        if (D > D_from_end(path.pid_dist)) {
            PID = true;
            pid_to_point(path.final_point, end_heading, cur_bz.max_power);
            return;
        }

        // Otherwise, GVF
        PID = false;
        set_velocity();
        if (cur_bz.interpolate_heading) set_turn_power(cur_bz.target_heading);
        else set_turn_power(Math.toDegrees(Utils.angle_v(path.derivative(D))));
        set_drive_power(turn_power);
        strafe_angle = get_strafe_angle();

        // Error
        error = closest_dist();

        // Drive according to calculations
        drive.drive_angle(strafe_angle, turn_power, drive_power, get_heading());
    }
}