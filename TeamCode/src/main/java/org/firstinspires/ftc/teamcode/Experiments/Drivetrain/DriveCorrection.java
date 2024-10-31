package org.firstinspires.ftc.teamcode.Experiments.Drivetrain;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDController;

@Config
public class DriveCorrection {
    public static double stable_hp = 0.0055, stable_hi = 0, stable_hd = 0.000035;
    public static double turn_hp = 0.0075, turn_hi = 0, turn_hd = 0.00003;

    WheelControl wheelControl;
    Odometry odometry;

    PIDController stable_correction;
    PIDController turn_correction;

    public DriveCorrection(WheelControl wheelControl, Odometry odometry){
        this.wheelControl = wheelControl;
        this.odometry = odometry;

        stable_correction = new PIDController(stable_hd, stable_hi, stable_hd);
        turn_correction = new PIDController(turn_hd, turn_hi, turn_hd);
    }

    public double stable_correction(double target_angle){
        return -stable_correction.calculate(odometry.opt.get_heading(), target_angle);
    }

    public double turn_correction(double target_angle){
        return -turn_correction.calculate(odometry.opt.get_heading(), target_angle);
    }
}
