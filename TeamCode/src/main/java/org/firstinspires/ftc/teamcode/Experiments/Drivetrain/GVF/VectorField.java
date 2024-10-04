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

    public VectorField(HardwareMap h, Odometry o, Path p, double speed){
        this.hardwareMap = h;
        this.odometry = o;
        this.path = p;

        this.D = 0.0;
        this.speed = speed;
        this.drive = new WheelControl(h, o);

    }

//    public Point get_v(Point p){
//        return this.path.get_v(p, this.speed);
//    }

    public Point calibrate(){
        // centripetal force and adequate power
        return new Point(1, 1);
    }

    public void update(double target_angle){
        Point v = path.get_v(new Point(odometry.getxPos(), odometry.getyPos()), speed);

        Point c = this.calibrate();
        double centripetal_f = c.x;
        double power = c.y;

        this.drive.drive(v.y, v.x, centripetal_f, target_angle, power);
        this.D += this.speed;
    }

    public void set_path(Path path){
        this.path = path;
    }

}
