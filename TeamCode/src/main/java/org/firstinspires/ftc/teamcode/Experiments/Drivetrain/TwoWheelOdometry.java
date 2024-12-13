package org.firstinspires.ftc.teamcode.Experiments.Drivetrain;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
//
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class TwoWheelOdometry {
    public static double threshold = 0.0002;

    public static double TICKS_TO_INCHES = 1.88976*Math.PI/(2048 * 0.982);
    //public static double TRACK_WIDTH = 10.1*0.987;
    public static double TRACK_LEFT = 5.4*1.1;
    //public static double TRACK_RIGHT = TRACK_WIDTH - TRACK_LEFT;
    public static double LENGTH = 3.2;

    public DcMotorEx verticalEncoder;
    public DcMotorEx horizontalEncoder;

    private double heading;
    private double xPos;
    private double yPos;

    double vert;
    double horiz;
    private double velocity;

    private double dX;
    private double dY;
    private double dTheta;

    private IMU imu;
    //public OptOdometry opt;

    ElapsedTime imuTimer = new ElapsedTime();

    public TwoWheelOdometry(HardwareMap hardwareMap, double heading, double x, double y, String vert, String horiz) {
        verticalEncoder = hardwareMap.get(DcMotorEx.class, vert);
        horizontalEncoder = hardwareMap.get(DcMotorEx.class, horiz);

        verticalEncoder.setDirection(DcMotorEx.Direction.REVERSE);
        horizontalEncoder.setDirection(DcMotorEx.Direction.REVERSE);

        this.vert = verticalEncoder.getCurrentPosition();
        this.horiz = -horizontalEncoder.getCurrentPosition();

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
        double oldV = vert; //set to prev update's values
        double oldH = horiz;
        double oldHeading = heading;

        vert = this.verticalEncoder.getCurrentPosition();
        horiz = -this.horizontalEncoder.getCurrentPosition();
        heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double dV = (vert-oldV)*TICKS_TO_INCHES;
        double dH = (horiz-oldH)*TICKS_TO_INCHES;

        dTheta = heading-oldHeading;

        // Y is FB and X is LR
        if (Math.abs(dTheta) <= threshold) { //prevents divide by 0 error (when driving straight forward)
            dY = dV;
            dX = dH;
            dTheta = 0;
        } else {
            dY = 2 * (dV / dTheta - TRACK_LEFT) * (Math.sin(dTheta / 2));
            dX = 2 * (dH / dTheta - LENGTH) * (Math.sin(dTheta / 2));
        }
        double change_x = dY*Math.sin(dTheta/2) + dX*Math.cos(dTheta/2);
        double change_y = dY*Math.cos(dTheta/2) + dX*Math.sin(dTheta/2);

        this.xPos = xPos+Math.cos(heading-Math.PI/2)*change_x-Math.sin(heading-Math.PI/2)*change_y;
        this.yPos = yPos+Math.sin(heading-Math.PI/2)*change_x+Math.cos(heading-Math.PI/2)*change_y;
    }

}
