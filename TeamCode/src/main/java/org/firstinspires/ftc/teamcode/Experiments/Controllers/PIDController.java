package org.firstinspires.ftc.teamcode.Experiments.Controllers;

import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.Utils;

import java.util.Objects;

public class PIDController {
    public double integralSum = 0;
    public double lastError = 0;
    public double error = 0;
    public PIDCoefficients coeff;
    public String mode;

    ElapsedTime timer;

    public PIDController(PIDCoefficients coeff, String mode) {
        timer = new ElapsedTime();
        this.coeff = coeff;
        this.mode = mode;
    }

    public void setCoefficients(PIDCoefficients coeff) {
        this.coeff = coeff;
    }

    public double calculate(double pos, double target) {
        error = target - pos;
        if (Objects.equals(mode, "Heading")) {
            error = Utils.limit_angle_deg(error);
        }

        integralSum += error*timer.seconds();

        double derivative = 0;
        if (lastError != 0) {
            derivative = (error - lastError) / timer.seconds();
        }

        double power = coeff.p*error + coeff.i*integralSum + coeff.d*derivative;

        lastError = error;

        this.timer.reset();

        return power;
    }
}
