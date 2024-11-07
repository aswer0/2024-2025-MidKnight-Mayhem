package org.firstinspires.ftc.teamcode.Experiments.Drivetrain;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Experiments.Utils.utils;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDController;

@Config
public class DriveCorrection {
    public static double stable_hp = 0.00022, stable_hi = 0, stable_hd = 0.00065;
    public static double turn_hp = 0.08, turn_hi = 0, turn_hd = 0.0005;

    double head;

    Odometry odometry;

    PIDController stable_correction;
    PIDController turn_correction;

    public DriveCorrection(Odometry odometry){
        this.odometry = odometry;
        this.head = 0.0;

        stable_correction = new PIDController(stable_hp, stable_hi, stable_hd);
        turn_correction = new PIDController(turn_hp, turn_hi, turn_hd);
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
        return wrap_correct(stable_correction.calculate(this.head, target_angle), target_angle, this.head);
    }

    public double turn_correction(double target_angle){
        return wrap_correct(turn_correction.calculate(this.head, target_angle), target_angle, this.head);
    }

    public void update_head(){
        this.head = utils.cap(odometry.opt.get_heading(), 0, 360);
    }
}
