package org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.opencv.core.Point;

public class VectorField {
    // Robot controls
    public Odometry odometry;
    WheelControl drive;
    Path path;

    // Robot tuning
    double max_speed;
    double min_speed;
    double max_turn_speed;
    double angle_to_power;
    double corr_weight;

    // Backend variables
    public double D;
    Point velocity = new Point(0, 0);
    public double speed;
    public double turn_speed;

    public VectorField(WheelControl w,
                       Odometry o,
                       Path p,
                       double max_speed,
                       double min_speed,
                       double max_turn_speed,
                       double angle_to_power,
                       double corr_weight) {
        this.odometry = o;
        this.path = p;
        this.drive = w;

        this.max_speed = max_speed;
        this.min_speed = min_speed;
        this.max_turn_speed = max_turn_speed;
        this.angle_to_power = angle_to_power;
        this.corr_weight = corr_weight;
    }

    public Point get_closest() {
        return path.forward(D);
    }

    public Point get_pos() {
        return new Point(odometry.opt.get_x(), odometry.opt.get_y());
    }

    // Updates closest point on curve using binary search
    public void update_closest(double look_ahead,
                               double max_rough_iters,
                               double tune_iters,
                               double rate) {
        Point pos = Utils.add_v(get_pos(), Utils.mul_v(velocity, look_ahead));
        double path_len = path.F[path.get_bz(D)].est_arclen;
        double update = rate*speed/path_len;

        int init_sign = path.dDdt_sign(pos, D);
        int iters = 0;
        while (path.dDdt_sign(pos, D) == init_sign && iters++ < max_rough_iters) {
            D -= init_sign*update;
        }
        for (int i = 0; i < tune_iters; i++) {
            if (path.dDdt_sign(pos, D) > 0) D -= update;
            else D += update;
            update /= 2;
        }
        if (D > path.n_bz) D = path.n_bz;
    }

    public double angle_to_path() {
        Point follow = path.forward(Math.min(D+0.05, path.n_bz));
        return Utils.angle_v(Utils.sub_v(follow, get_pos()));
    }

    public void move() {
        update_closest(0, 50, 5, 1);
        if (D == path.n_bz/* && Utils.length(Utils.sub_v(get_pos(), get_closest())) < 100*/) return;
        double target_angle = angle_to_path();
        turn_speed = odometry.opt.get_heading()-Math.toDegrees(target_angle);
        if (turn_speed < -180) turn_speed += 360;
        if (turn_speed > 180) turn_speed -= 360;
        turn_speed /= angle_to_power;
        if (turn_speed > max_turn_speed) turn_speed = max_turn_speed;
        if (turn_speed < -max_turn_speed) turn_speed = -max_turn_speed;
        speed = min_speed+(turn_speed/max_turn_speed)*(min_speed-max_speed);
        velocity = Utils.scale_v(new Point(Math.cos(target_angle), Math.sin(target_angle)), speed);
        drive.drive(1, 0, turn_speed, 0, speed);
    }
}
