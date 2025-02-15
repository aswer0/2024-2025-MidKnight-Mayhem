package org.firstinspires.ftc.teamcode.Experiments.Controllers;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class ScheduledPID {
    public int current_pid;
    public PIDController primary;
    public PIDController secondary;
    public double secondary_dist;
    public boolean reset_i;

    public ScheduledPID(PIDCoefficients primary,
                        PIDCoefficients secondary,
                        String mode,
                        double secondary_dist,
                        boolean reset_i) {
        current_pid = 1;
        this.primary = new PIDController(primary, mode);
        this.secondary = new PIDController(secondary, mode);
        this.secondary_dist = secondary_dist;
        this.reset_i = reset_i;
    }

    public double calculate(double pos, double target) {
        if (Math.abs(target - pos) > secondary_dist) {
            if (current_pid == 2) {
                if (reset_i) {
                    secondary.integralSum = 0;
                } else {
                    secondary.integralSum = primary.integralSum;
                }
                primary.lastError = target - pos;
                current_pid = 1;
            }
            return primary.calculate(pos, target);
        } else {
            if (current_pid == 1) {
                if (reset_i) {
                    primary.integralSum = 0;
                } else {
                    secondary.integralSum = primary.integralSum;
                }
                secondary.lastError = target - pos;
                current_pid = 2;
            }
            return secondary.calculate(pos, target);
        }
    }
}