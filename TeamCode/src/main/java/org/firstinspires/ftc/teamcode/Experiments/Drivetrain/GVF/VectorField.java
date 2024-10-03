package org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.opencv.core.Point;

public class VectorField {
    // vector field class
    HardwareMap hardwareMap;
    Odometry odometry;
    Path path;

    public VectorField(HardwareMap h, Odometry o, Path p){
        this.hardwareMap = h;
        this.odometry = o;
        this.path = p;
    }

    public Point get_v(double x, double y){
        return new Point(0, 0);
    }

    public void calibrate(){

    }

    public void update(){

    }

    public void set_path(Path path){
        this.path = path;
    }

}
