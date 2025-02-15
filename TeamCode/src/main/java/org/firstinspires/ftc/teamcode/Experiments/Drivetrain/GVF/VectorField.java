package org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.Experiments.Controllers.TestPID;
import org.firstinspires.ftc.teamcode.Experiments.Controllers.HPIDController;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.opencv.core.Point;

public class VectorField {
    // Robot controls
    public Odometry odometry;
    public WheelControl drive;
    public BCPath path;

    // Motion profiling
    double velocity_update_rate = 0.1;
    double p_to_v = 68;
    public Point prev_pos;
    public double true_speed = 0;
    public Point velocity = new Point(0, 0);

    // Speed tuning
    double max_speed = 1;
    double min_speed = 0.8;
    double max_turn_speed = 0.5;

    // Correction constants
    double path_corr = 0.1;
    double centripetal_corr = 0.02;
    //double centripetal_threshold = 10;
    double accel_corr = 0;

    // PID constants (at end of path)
    double PID_dist = 10;
    double stop_speed = 0.2;
    double stop_decay = 0.003;
    public static double end_decel = 0.05;
    public Point end_target;

    // Heading controls
    boolean path_heading;
    double end_heading;

    // Backend variables
    public double T;
    public Point powers = new Point(0, 0);
    public double speed;
    public double turn_speed;
    public boolean PID = false;
    public double error = 0;
    public ElapsedTime timer;

    // PID variables
    public double xp = end_decel, xi = 0.1, xd = 0.01, xithres = 2;
    public double yp = end_decel, yi = 0.1, yd = 0.01, yithres = 2;
    public double hp = 0.04, hi = 0.025, hd = 0.003, hithres = 3;

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
        x_PID = new TestPID(xp, xi, xd, xithres);
        y_PID = new TestPID(yp, yi, yd, yithres);
        h_PID = new HPIDController(hp, hi, hd, hithres);

        // Timer
        timer = new ElapsedTime();
        prev_pos = get_pos();
    }

    public void setPath(BCPath path, double end_heading, boolean path_heading) {
        this.T = 0;
        this.path = path;
        this.end_heading = end_heading;
        this.path_heading = path_heading;
    }

    // Gets closest point on path to robot
    public Point get_closest() {
        return path.forward(T);
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

    public boolean at_point(Point p, double threshold) {
        return Utils.dist(get_pos(), p) <= threshold;
    }

    public boolean at_angle_deg(double h, double threshold) {
        return Utils.limit_angle_deg(h-get_heading()) < threshold;
    }

    public boolean at_angle_rad(double h, double threshold) {
        return Utils.limit_angle_rad(h-Math.toRadians(get_heading())) < threshold;
    }

    // Sets velocity of robot
    public void set_velocity() {
        if (timer.seconds() < velocity_update_rate) return;
        velocity = Utils.div_v(Utils.sub_v(get_pos(), prev_pos), timer.seconds());
        true_speed = Utils.len_v(velocity);
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
    public void update_closest(double look_ahead,
                               double max_rough_iters,
                               double tune_iters,
                               double rate) {
        Point pos = Utils.add_v(get_pos(), Utils.mul_v(powers, look_ahead));
        double path_len = path.F[path.get_bz(T)].est_arclen;
        double update = rate*speed/path_len;

        // Get rough estimate
        int init_sign = path.dDdt_sign(pos, T);
        int iters = 0;
        while (path.dDdt_sign(pos, T) == init_sign && iters++ < max_rough_iters) {
            T -= init_sign*update;
        }

        // Binary search to tune closest
        for (int i = 0; i < tune_iters; i++) {
            if (path.dDdt_sign(pos, T) > 0) T -= update;
            else T += update;
            update /= 2;
        }
        if (T > path.n_bz) T = path.n_bz;
    }

    // Calculates T (parameter) value that is approximately "dist" distance from end
    public double T_from_end(double dist) {
        return path.n_bz-dist/path.F[path.n_bz-1].est_arclen;
    }

    // Calculates distance to endpoint
    public double dist_to_end() {
        return Utils.dist(end_target, get_pos());
    }

    // Distance to closest point on path
    public double closest_dist() {
        return Utils.dist(path.forward(T), get_pos());
    }

    // Gets speed when approaching end
    public double get_end_speed(Point p) {
        return end_decel*Utils.dist(get_pos(), p);
    }

    // Calculates turn speed based on target angle
    public void set_turn_speed(double target_angle) {
        turn_speed = h_PID.calculate(get_heading(), target_angle);
        if (turn_speed > max_turn_speed) turn_speed = max_turn_speed;
        if (turn_speed < -max_turn_speed) turn_speed = -max_turn_speed;
    }

    // Robot's move vector to path
    public Point move_vector(double speed) {
        update_closest(0, 50, 5, 1);

        // Base vector (orthogonal & tangent)
        Point orth = Utils.mul_v(Utils.sub_v(get_closest(), get_pos()), path_corr);
        Point tangent = Utils.scale_v(path.derivative(T), 1);
        Point move_v = Utils.add_v(orth, tangent);

        // Acceleration correction
        //Point accel_corr_term = get_accel_corr(move_v);

        // Centripetal correction
        Point centripetal = new Point(0, 0);
        if (centripetal_corr > 0) {
            double perp_angle = Utils.angle_v_rad(tangent)+Math.PI/2;
            double centripetal_len = path.curvature(T)*centripetal_corr*true_speed*closest_dist()/p_to_v;
            centripetal = Utils.polar_to_rect_rad(centripetal_len, perp_angle);
        }

        // Add everything
        return Utils.add_v(Utils.scale_v(move_v, speed), centripetal);
        //return Utils.add_v(Utils.scale_v(move_v, speed), accel_corr_term);
    }

    // Move to a point given coordinates and heading
    public void pid_to_point(Point p, double target_heading, double max_speed) {
        end_target = p;

        // PID errors
        x_error = x_PID.calculate(get_x(), p.x);
        y_error = y_PID.calculate(get_y(), p.y);
        turn_speed = h_PID.calculate(get_heading(), target_heading);

        // Drive
        drive.drive_limit_power(x_error, y_error, turn_speed, max_speed, get_heading());
    }

    public void set_drive_speed(double turn_speed) {
        speed = max_speed;
        //speed = max_speed-(turn_speed/max_turn_speed)*(max_speed-min_speed);
        speed = Math.min(speed, get_end_speed(path.final_point));
    }

    // Move with GVF
    public void move() {
        // Set end target
        end_target = path.final_point;

        // PID at the end
        if (T > T_from_end(PID_dist)) {
            PID = true;
            pid_to_point(path.final_point, end_heading, max_speed);
            return;
        }

        // Otherwise, GVF
        PID = false;
        set_velocity();
        if (!path_heading) set_turn_speed(end_heading);
        else set_turn_speed(Utils.angle_v_deg(path.derivative(T)));
        set_drive_speed(turn_speed);
        powers = move_vector(speed);

        // Error
        error = Utils.dist(get_pos(), path.forward(T));

        // Drive according to calculations
        drive.drive(-powers.x, -powers.y, turn_speed, Math.toRadians(get_heading()), 1);
    }
}