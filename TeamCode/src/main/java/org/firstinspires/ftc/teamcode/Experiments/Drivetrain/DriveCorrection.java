package org.firstinspires.ftc.teamcode.Experiments.Drivetrain;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Experiments.Utils.utils;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDController;

import java.util.Optional;

@Config
public class DriveCorrection {
    public static double stable_hp = 0.00015, stable_hi = 0, stable_hd = 0.001;
    public static double set_hp = 0.1, set_hi = 0, set_hd = 0.001;

    Odometry odometry;

    PIDController stable_correction;
    PIDController set_correction;

    public DriveCorrection(Odometry odometry){
        this.odometry = odometry;

        stable_correction = new PIDController(stable_hp, stable_hi, stable_hd);
        set_correction = new PIDController(set_hd, set_hi, set_hp);
    }

    public double stable_correction(double target_angle){
        return stable_correction.calculate(odometry.opt.get_heading_unnorm(), target_angle);
    }

    public double set_correction(double target_angle){
        return set_correction.calculate(odometry.opt.get_heading_unnorm(), target_angle);
    }
}
