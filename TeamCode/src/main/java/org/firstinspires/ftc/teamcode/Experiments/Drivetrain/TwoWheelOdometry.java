package org.firstinspires.ftc.teamcode.Experiments.Drivetrain;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVFNew.Utils;

@Config
public class TwoWheelOdometry {
    public static double threshold = 0.0002;
    public static double TICKS_TO_INCHES = 1.88976*Math.PI/(2048 * 0.982);

    // Offsets of encoders
    // Left to right:  negative to positive x
    // Down to up: negative to positive y
    public static double vertical_encoder_x = -5.26;
    public static double horizontal_encoder_y = 1.98;

    // Encoders
    public DcMotorEx vertical_encoder;
    public DcMotorEx horizontal_encoder;
    public IMU imu;
    public OptOdometry opt;

    // Odometry of robot
    private double heading;
    private double x_pos;
    private double y_pos;

    // Positions of encoders
    double v_pos;
    double h_pos;

    // Change in values
    private double d_v;
    private double d_h;
    private double d_heading;
    private double d_x;
    private double d_y;

    public TwoWheelOdometry(HardwareMap hardwareMap,
                            double start_heading,
                            double start_x,
                            double start_y,
                            String vertical_string,
                            String horizontal_string) {
        vertical_encoder = hardwareMap.get(DcMotorEx.class, vertical_string);
        horizontal_encoder = hardwareMap.get(DcMotorEx.class, horizontal_string);

        vertical_encoder.setDirection(DcMotorEx.Direction.REVERSE);
        horizontal_encoder.setDirection(DcMotorEx.Direction.REVERSE);

        v_pos = vertical_encoder.getCurrentPosition();
        h_pos = horizontal_encoder.getCurrentPosition();

        heading = start_heading;
        x_pos = start_x;
        y_pos = start_y;

        // Set up IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();
    }

    public double getXPos() {
        return this.x_pos;
    }

    public double getYPos() {
        return this.y_pos;
    }

    public double getHeadingRad() {
        return this.heading;
    }

    public double getHeadingDeg() {
        return Math.toDegrees(this.heading);
    }

    public void setPos(double x, double y, double theta) {
        x_pos = x;
        y_pos = y;
        heading = theta;
    }

    public void update() {
        // Previous values
        double old_v = v_pos;
        double old_h = h_pos;
        double old_heading = heading;

        // Current values
        heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        v_pos = this.vertical_encoder.getCurrentPosition();
        h_pos = this.horizontal_encoder.getCurrentPosition();

        // Change in values
        d_v = (v_pos-old_v)*TICKS_TO_INCHES;
        d_h = (h_pos-old_h)*TICKS_TO_INCHES;
        d_heading = Utils.limit_angle_rad(heading-old_heading);

        // Calculate dx and dy relative to robot
        if (Math.abs(d_heading) <= threshold) {
            d_x = d_v;
            d_y = d_h;
        } else {
            double coeff = 2*Math.sin(d_heading/2);
            d_x = coeff * (d_v / d_heading - vertical_encoder_x);
            d_y = coeff * (d_h / d_heading + horizontal_encoder_y);
        }

        // Calculate new position
        double middle_heading = old_heading+d_heading/2;
        double d_x_field = d_x*Math.cos(middle_heading) - d_y*Math.sin(middle_heading);
        double d_y_field = d_y*Math.sin(middle_heading) + d_x*Math.cos(middle_heading);
        this.x_pos += d_x_field;
        this.y_pos += d_y_field;
    }
}