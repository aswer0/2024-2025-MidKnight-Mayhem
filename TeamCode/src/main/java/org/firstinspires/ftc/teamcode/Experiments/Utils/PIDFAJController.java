package org.firstinspires.ftc.teamcode.Experiments.Utils;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDFAJController {
    private double kp, ki, kd, kf, ka, kj;
    private double integralSum = 0;
    private double lastError = 0;
    private double lastDerivative = 0;
    private double lastAcceleration = 0;

    private double max_accel;

    ElapsedTime timer;

    public PIDFAJController(double kp, double ki, double kd, double kf, double ka, double kj) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;
        this.ka = ka;
        this.kj = kj;
        timer = new ElapsedTime();
    }

    public void setConstants(double kp, double ki, double kd, double kf, double ka, double kj) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;
        this.ka = ka;
        this.kj = kj;
    }

    public double calculate(double pos, double target) {
        double error = target - pos;
        double dt = timer.seconds();

        integralSum = integralSum + (error * dt);

        double derivative = (error - lastError) / timer.seconds();

        double targetAccel = Math.min(this.max_accel, Math.abs(error) / dt);
        double acceleration = (derivative - lastDerivative) / dt;
        double jerk = (acceleration - lastAcceleration) / dt;

        double power =
                this.kp * error +
                this.ki * integralSum +
                this.kd * derivative +
                (Math.signum(error) == -1 ? this.kf : 0) +
                this.ka * targetAccel +
                this.kj * jerk;

        lastError = error;
        lastDerivative = derivative;
        lastAcceleration = acceleration;

        this.timer.reset();

        return power;
    }

    public void set_max_accel(double max_accel){
        this.max_accel = max_accel;
    }
}
