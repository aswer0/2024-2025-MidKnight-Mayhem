package org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVFNew;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.Experiments.Controllers.HPIDController;
import org.firstinspires.ftc.teamcode.Experiments.Controllers.TestPID;
import org.opencv.core.Point;

public class VectorField {
    // Robot controls
    public Odometry odometry;
    public WheelControl drive;
    public CompositePath path;

    // Motion profiling
    double velocity_update_delay = 0.1;
    double p_to_v = 65;
    public Point prev_pos;
    public double speed = 0;
    public Point velocity = new Point(0, 0);

    // Correction constants
    double path_corr = 0.1;
    double centripetal_corr = 0;

    // PID constants (at end of path)
    double end_decel = 0.1;
    Point end_target;

    // Heading controls
    double end_heading;

    // Backend variables
    public double T;
    public BezierPath cur_bz;
    public double strafe_angle;
    public double drive_power;
    public double turn_power;
    public boolean PID = false;
    public double error = 0;
    public ElapsedTime timer;

    // PID variables
    public double xp = end_decel, xi = 0.1, xd = 0.01, xithres = 2;
    public double yp = end_decel, yi = 0.1, yd = 0.01, yithres = 2;
    public double hp = 0.02, hi = 0.025, hd = 0.002, hithres = 3;

    public TestPID x_PID;
    public TestPID y_PID;
    public HPIDController h_PID;

    public double x_error;
    public double y_error;

    // Constructor
    public VectorField(WheelControl w,
                       Odometry o) {
        // Inputs
        this.odometry = o;
        this.drive = w;

        // Zero power behavior: brake
        drive.change_mode(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set PID controllers
        this.x_PID = new TestPID(xp, xi, xd, xithres);
        this.y_PID = new TestPID(yp, yi, yd, yithres);
        this.h_PID = new HPIDController(hp, hi, hd, hithres);

        // Timer
        timer = new ElapsedTime();
        prev_pos = get_pos();
        
        // Closest parameter value
        this.T = 0;
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

    /*// Sets velocity measurement of robot
    public void set_velocity() {
        if (timer.seconds() < velocity_update_delay) return;
        velocity = Utils.div_v(Utils.sub_v(get_pos(), prev_pos), timer.seconds());
        speed = Utils.len_v(velocity);
        prev_pos = get_pos();
        timer.reset();
    }

    // Updates closest point on curve using signed GD & binary search
    public void update_closest(double max_rough_iters,
                               double tune_iters,
                               double rate) {
        Point pos = get_pos();
        double path_len = path.get_bz(T).total_arclen;
        double update = rate*drive_power/path_len;

        // Get rough estimate
        int iters = 0;
        int init_sign = path.dDdt_sign(pos, T);
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
    public double D_from_end(double dist) {
        BezierPath last_bz = path.F[path.n_bz-1];
        return path.n_bz-1+last_bz.dist_to_t(last_bz.total_arclen-dist);
    }*/

    // Calculates distance to endpoint
    public double dist_to_end() {
        return Utils.dist(end_target, get_pos());
    }

    /*// Distance to closest point on path
    public double closest_dist() {
        return Utils.dist(path.forward(T), get_pos());
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
    public void set_strafe_angle() {
        // Base vector (orthogonal & tangent)
        Point orth = Utils.mul_v(Utils.sub_v(get_closest(), get_pos()), path_corr);
        Point tangent = Utils.scale_v(path.derivative(T), 1);
        Point move_v = Utils.add_v(orth, tangent);

        // Centripetal correction
        Point centripetal_v = new Point(0, 0);
        if (centripetal_corr > 0) {
            double perp_angle = Utils.angle_v_rad(tangent)+Math.PI/2;
            double centripetal_len = path.curvature(T)*centripetal_corr*speed/p_to_v;
            centripetal_v = Utils.polar_to_rect_rad(centripetal_len, perp_angle);
        }

        // Add everything
        strafe_angle = Utils.angle_v_deg(Utils.add_v(move_v, centripetal_v));
    }*/

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

    /*public void set_drive_power() {
        double max_p = cur_bz.max_power, min_p = cur_bz.min_power;
        double curvature = path.curvature(T);
        drive_power = Math.max(max_p-curvature*cur_bz.curvature_power_decay, min_p);
        drive_power = Math.min(drive_power, get_end_power(path.final_point));
    }

    // Move with GVF
    public void follow(CompositePath path) {
        if (this.path != path) {
            T = 0; this.path = path;
        }

        end_target = path.final_point;

        update_closest(50, 5, 1);
        cur_bz = path.get_bz(T);
        
        // PID at the end
        if (T > D_from_end(path.pid_dist)) {
            PID = true;
            pid_to_point(path.final_point, end_heading, cur_bz.max_power);
            return;
        }

        // Otherwise, GVF
        PID = false;
        set_velocity();
        set_turn_power(path.get_target_heading(T));
        set_drive_power();
        set_strafe_angle();

        // Error
        error = closest_dist();

        // Drive according to calculations
        drive.drive_angle(strafe_angle, turn_power, drive_power, get_heading());
    }*/
}