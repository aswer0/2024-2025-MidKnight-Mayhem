package org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.opencv.core.Point;

public class VectorField {
    // vector field class
    HardwareMap hardwareMap;
    Odometry odometry;
    Path path;
    WheelControl drive;

    double D;
    double speed;

    public VectorField(WheelControl w, Odometry o, Path p, double speed){
        this.drive = w;
        this.odometry = o;
        this.path = p;

        this.D = 0.0;
        this.speed = speed;

    }

    public Point get_v(double x, double y){
        int bz = (int)Math.floor(this.D);
        return this.path.F.get(bz).get_v(new Point(x, y), this.speed);
    }

    public Point calibrate(){
        // Centripetal force and adequate power

        return new Point(1, 1);
    }

    public void update(double target_angle){
        Point p = this.get_v(odometry.getxPos(), odometry.getyPos());
        double x = p.x;
        double y = p.y;

        Point c = this.calibrate();
        double centripetal_f = c.x;
        double power = c.y;

        this.drive.drive(y, x, centripetal_f, target_angle, power);
        this.D += this.speed;
    }

    public void set_path(Path path){
        this.path = path;
    }

}
