package org.firstinspires.ftc.teamcode.Experiments.Utils;

import com.qualcomm.robotcore.util.ElapsedTime;

public class HPIDController {
    private double kp, ki, kd, ithres;
    private double integralSum = 0;
    private double lastError = 0;

    ElapsedTime timer;

    public HPIDController(double kp, double ki, double kd, double ithres) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.ithres = ithres;
        timer = new ElapsedTime();
    }

    public void setConstants(double kp, double ki, double kd, double ithres) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.ithres = ithres;
    }

    public double calculate(double pos, double target) {
        double error = target - pos;
        error %= 360;
        if (error > 180) error -= 360;

        if (error < this.ithres) {
            integralSum += error*timer.seconds();
        } else integralSum = 0;

        integralSum = integralSum + (error * timer.seconds());

        double derivative = (error - lastError) / timer.seconds();

        double power = this.kp*error + this.ki*integralSum + this.kd*derivative;

        lastError = error;

        this.timer.reset();

        return power;
    }
}
