package org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDController;
import org.opencv.core.Point;
//
public class GVFPID {
    // Robot controls
    public Odometry odometry;
    WheelControl drive;
    Path path;

    // Robot tuning
    double max_speed;
    double min_speed;

    // End parameters
    double end_decel = 0.003;
    double end_heading;

    // Backend variables
    public double D = 0.0;
    Point velocity = new Point(0, 0);
    public double speed;
    public double turn_speed;

    // PID at end of path
    double xp = 0.022, xi = 0, xd = 0;
    double yp = 0.022, yi = 0, yd = 0;
    double hp = 0.011, hi = 0, hd = 0.0001;
    PIDController x_PID;
    PIDController y_PID;
    PIDController heading_PID;

    // Constructor
    public GVFPID(WheelControl w,
                  Odometry o,
                  Path p,
                  double max_speed,
                  double min_speed,
                  double end_heading) {
        // Set robot IO
        this.odometry = o;
        this.path = p;
        this.drive = w;

        // Hyperparameters
        this.max_speed = max_speed;
        this.min_speed = min_speed;
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
        //if (D > path.n_bz) D = path.n_bz;
    }

    // PID to a point given coordinates and heading
    public void pid_to_point(Point p, double target_angle, double power) {
        double x_error = x_PID.calculate(get_x(), p.x);
        double y_error = y_PID.calculate(get_y(), p.y);
        double head_error = heading_PID.calculate(get_heading(), target_angle);
        drive.drive(x_error, -y_error, -head_error, -Math.toRadians(get_heading()), power);
    }

    // Move with GVF and PID at the end
    public void move() {
        update_closest(0, 50, 5, 1);
        if (D == path.n_bz && Utils.dist(get_pos(), get_closest()) < 10) {
            pid_to_point(path.forward(D), end_heading, 0.3);
        }
        double follow_D = Math.min(D+0.01, path.n_bz);
        Point derivative = path.derivative(follow_D);
        double follow_h = Math.toDegrees(Math.atan2(derivative.y, derivative.x));
        pid_to_point(path.forward(follow_D), follow_h, 0.4);
    }
}