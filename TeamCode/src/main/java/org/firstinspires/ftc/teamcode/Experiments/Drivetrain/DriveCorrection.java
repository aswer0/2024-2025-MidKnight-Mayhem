package org.firstinspires.ftc.teamcode.Experiments.Drivetrain;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Experiments.Utils.utils;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDController;

import java.util.Optional;

@Config
public class DriveCorrection {
    public static double stable_hp = 0.00015, stable_hi = 0, stable_hd = 0.001;

    double ta = -180;

    Odometry odometry;

    PIDController stable_correction;

    public DriveCorrection(Odometry odometry){
        this.odometry = odometry;

        stable_correction = new PIDController(stable_hp, stable_hi, stable_hd);
    }

    public double wrap_correct(double error, double target, double actual){
        if (Math.abs((target % 360) - (actual % 360)) < 180){
            return error;
        }
        else {
            return (-error) % 360;
        }
    }

    public double stable_correction(double target_angle){
        return wrap_correct(
                stable_correction.calculate(odometry.opt.get_heading(), target_angle),
                target_angle,
                odometry.opt.get_heading()
        );
    }
    public void set_target_angle(double target_angle){
        this.ta = target_angle;
    }
}
