package org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDController;
import org.opencv.core.Point;

public class VectorField {
    // Robot controls
    public Odometry odometry;
    WheelControl drive;
    Path path;

    // Robot tuning
    double max_speed = 0.7;
    double min_speed = 0.5;
    double max_turn_speed = 20;
    double angle_to_power = 100;
    double corr_weight = 0.1;

    //
    double target_angle;

    // End decel: speed decrease per distance
    double end_decel = 0.02;
    double end_heading;

    // Backend variables
    public double D;
    public Point velocity = new Point(0, 0);
    public double speed;
    public double turn_speed;
    public boolean PID = false;
    public double error = 0;

    // PID at end of path.
    double xp = 0.1, xi = 0, xd = 0;
    double yp = 0.1, yi = 0, yd = 0;
    double hp = 0.02, hi = 0, hd = 0.0001;

    // Constructor
    public VectorField(WheelControl w,
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
    public Point move_vector(double speed) {
        update_closest(0, 50, 5, 1);
        if (D > path.n_bz-0.1) {
            PID = true;
            return Utils.scale_v(Utils.sub_v(path.final_point, get_pos()), speed);
        }
        PID = false;
        Point orth = Utils.sub_v(get_closest(), get_pos());
        orth = Utils.scale_v(orth, corr_weight*Utils.length(orth));
        Point tangent = Utils.scale_v(path.derivative(D), 1);
        return Utils.scale_v(Utils.add_v(orth, tangent), speed);
    }

    public void set_turn_speed(double target_angle) {
        turn_speed = turn_angle(get_heading(), target_angle)/angle_to_power;
        if (turn_speed > max_turn_speed) turn_speed = max_turn_speed;
        if (turn_speed < -max_turn_speed) turn_speed = -max_turn_speed;
    }

    // Move to a point given coordinates and heading
    public void move_to_point(Point p, double target_angle, double power) {
        // Get speed and movement
        double end_speed = end_decel*Utils.dist(get_pos(), p)+0.05;
        speed = Math.min(end_speed, power);
        velocity = Utils.scale_v(Utils.sub_v(p, get_pos()), speed);

        // Turn speed
        set_turn_speed(target_angle);

        // Drive
        drive.drive(velocity.x, -velocity.y, -turn_speed, -Math.toRadians(get_heading()), 1);
    }

    // Move with GVF and PID at the end
    public void move() {
        // Get speed with curves and end decel
        double drive_speed = min_speed+(turn_speed/max_turn_speed)*(max_speed-min_speed);
        double end_speed = end_decel*Utils.dist(get_pos(), path.final_point)+0.05;
        speed = Math.min(drive_speed, end_speed);
        velocity = move_vector(speed);

        // Error
        error = Utils.dist(get_pos(), path.forward(D));

        // Angle
        double target_angle = end_heading;
        if (D < path.n_bz-0.2) target_angle = Math.toDegrees(Utils.angle_v(path.derivative(D)));
        set_turn_speed(target_angle);

        // Drive according to calculations
        drive.drive(velocity.x, -velocity.y, -turn_speed, -Math.toRadians(get_heading()), 1);
    }
}