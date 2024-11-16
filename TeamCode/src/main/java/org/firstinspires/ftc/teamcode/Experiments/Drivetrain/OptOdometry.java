package org.firstinspires.ftc.teamcode.Experiments.Drivetrain;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class OptOdometry {
    private double heading;
    private double xPos;
    private double yPos;


    //TEST ROBOT OFFSET:
    //X: 2.296
    //Y: -5.709
    //X: 90
    //target/actual
    //3600/3601.225
    //137.375/139

    //

    private static double SENSOR_OFFSET_X = 0;
    private static double SENSOR_OFFSET_Y = 0;
    private static double SENSOR_OFFSET_H = 90;

    private static double LINEAR_SCALAR = 1.011530906853465;
    private static double ANGULAR_SCALAR = 0.994178050664664;

    SparkFunOTOS otos;

    public OptOdometry(HardwareMap hardwareMap, double h, double x, double y, String oto){
        otos = hardwareMap.get(SparkFunOTOS.class, oto);
        this.xPos = x;
        this.yPos = y;
        this.heading = h;

        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);
        otos.setLinearScalar(LINEAR_SCALAR);
        otos.setAngularScalar(ANGULAR_SCALAR);

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(SENSOR_OFFSET_X, SENSOR_OFFSET_Y, SENSOR_OFFSET_H);
        otos.setOffset(offset);

        otos.calibrateImu();
        otos.resetTracking();
        otos.setPosition(new SparkFunOTOS.Pose2D(this.xPos, this.yPos, this.heading));
    }

    public double get_x() {
        return this.xPos;
    }

    public double get_y() {
        return this.yPos;
    }

    public double get_heading() {
        return this.heading;
    }

    public void update(){
        SparkFunOTOS.Pose2D pos = this.otos.getPosition();
        this.xPos = pos.x;
        this.yPos = pos.y;
        this.heading = pos.h % 360;
    }

    public void setPos(double x, double y){
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        this.otos.setPosition(currentPosition);
    }

    public void calibrate_imu(){
        this.otos.calibrateImu();
    }

    public void calibrate_tracking(){
        this.otos.resetTracking();
    }

    public double get_linear_scalar(){
        return LINEAR_SCALAR;
    }
    public double get_angular_scalar(){
        return ANGULAR_SCALAR;
    }

}
