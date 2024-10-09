package org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.opencv.core.Point;

public class VectorField {
    // Vector field class
    Odometry odometry;
    Path path;
    WheelControl drive;

    double D;
    double speed;

    public VectorField(WheelControl w, Odometry o, Path p, double speed){
        this.odometry = o;
        this.path = p;

        this.D = 0.0;
        this.speed = speed;
        this.drive = w;

    }

    public Point calibrate(){
        // centripetal force and adequate power
        return new Point(0.35, 3.2);
    }

    public void update(){
        Point v = path.get_v(new Point(odometry.getxPos(), odometry.getyPos()), speed);

        Point c = this.calibrate();
        double centripetal_f = c.x;
        double power = c.y;

        //this.drive.drive(v.y, v.x, centripetal_f, target_angle, power);
        this.D += this.speed;
    }

    public void set_path(Path path){
        this.path = path;
    }

}
