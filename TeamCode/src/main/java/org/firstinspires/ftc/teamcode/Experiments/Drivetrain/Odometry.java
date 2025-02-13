package org.firstinspires.ftc.teamcode.Experiments.Drivetrain;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class Odometry {
    public static double threshold = 0.0002;

    public static  double TICKS_TO_INCHES = 1.88976*Math.PI/(2048 * 0.982);
    public static  double TRACK_WIDTH = 10.1*0.987;
    public static  double TRACK_LEFT = 5.4*1.1;
    public static  double TRACK_RIGHT = TRACK_WIDTH - TRACK_LEFT;
    public static  double LENGTH = 3.2;

    public DcMotorEx leftEncoder;
    public DcMotorEx horizontalEncoder;
    public DcMotorEx rightEncoder;

    private double heading;
    private double xPos;
    private double yPos;

    double right;
    double left;
    double front;
    private double velocity;

    private double dX;
    private double dY;
    private double dTheta;

    private IMU imu;
    public OptOdometry opt;
    public TwoWheelOdometry two;

    ElapsedTime imuTimer = new ElapsedTime();

    public Odometry() {}

    public Odometry(HardwareMap hardwareMap, double heading, double x, double y, String left, String right, String front) {
        leftEncoder = hardwareMap.get(DcMotorEx.class, left);
        rightEncoder = hardwareMap.get(DcMotorEx.class, right);
        horizontalEncoder = hardwareMap.get(DcMotorEx.class, front);

        leftEncoder.setDirection(DcMotorEx.Direction.REVERSE);
        horizontalEncoder.setDirection(DcMotorEx.Direction.REVERSE);

        this.left = leftEncoder.getCurrentPosition();
        this.right = rightEncoder.getCurrentPosition();
        this.front = -horizontalEncoder.getCurrentPosition();

        this.heading = heading;
        this.xPos = x;
        this.yPos = y;

        this.imu = hardwareMap.get(IMU.class,"imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        imu.resetYaw();
        imuTimer.reset();
    }

    public Odometry(HardwareMap hardwareMap, double heading, double x, double y, String otos){
        opt = new OptOdometry(hardwareMap, heading, x, y, otos);
    }

    public void addTwoWheelOdometry(HardwareMap hardwareMap,
                                    double start_heading,
                                    double start_x,
                                    double start_y,
                                    String vertical_string,
                                    String horizontal_string) {
        two = new TwoWheelOdometry(hardwareMap, start_heading, start_x, start_y, vertical_string, horizontal_string);
    }

    public double getxPos() {
        return this.xPos;
    }

    public double getyPos() {
        return this.yPos;
    }

    public double getHeading() {
        return this.heading;
    }

    public void setPos(double x, double y, double theta) {
        xPos = x;
        yPos = y;
        heading = theta;
    }
    public void update() {
        boolean useIMU = false;
//        if (imuTimer.milliseconds()>500) { //update imu heading everyone 500 milliseconds
//            heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//            useIMU = true;
//            imuTimer.reset();
//        } else {
//            useIMU = false;
//        }

        double oldL = left; //set to prev update's values
        double oldR = right;
        double oldF = front;

        left = this.leftEncoder.getCurrentPosition();
        right = this.rightEncoder.getCurrentPosition();
        front = -this.horizontalEncoder.getCurrentPosition();

        double dL = (left-oldL)*TICKS_TO_INCHES;
        double dR = (right-oldR)*TICKS_TO_INCHES;
        double dF = (front-oldF)*TICKS_TO_INCHES;

        if (useIMU) {
            double cur_heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            dTheta = cur_heading-heading;
            heading = cur_heading;
            imuTimer.reset();
        }
        dTheta = (dL - dR) / (TRACK_WIDTH);

        if (Math.abs(dTheta) <= threshold) { //prevents divide by 0 error (when driving straight forward)
            dY = (dR+dL)/2;
            dX = dF;
            dTheta = 0;
        } else {
            dY = 2 * (dR / dTheta + TRACK_RIGHT) * (Math.sin(dTheta / 2));
            dX = 2 * (dF / dTheta - LENGTH) * (Math.sin(dTheta / 2));
        }
        /*double change_x = dY*Math.sin(dTheta/2) + dX*Math.cos(dTheta/2);
        double change_y = dY*Math.cos(dTheta/2) + dX*Math.sin(dTheta/2);

        this.xPos = xPos + Math.cos(heading-Math.PI/2)*change_x-Math.sin(heading-Math.PI/2)*change_y;
        this.yPos = yPos + Math.sin(heading-Math.PI/2)*change_x+Math.cos(heading-Math.PI/2)*change_y;*/
        this.xPos = xPos + Math.cos(heading-Math.PI/2)*dX-Math.sin(heading-Math.PI/2)*dY;  //globalization
        this.yPos = yPos + Math.sin(heading-Math.PI/2)*dX+Math.cos(heading-Math.PI/2)*dY;
        if (!useIMU) this.heading = heading - dTheta;
    }

}
