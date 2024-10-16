package org.firstinspires.ftc.teamcode.Experiments.Utils;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDFController {
    public PIDFCoefficients coefficients;
    ElapsedTime elapsedTime = new ElapsedTime();
    double iSum;
    double lastError;
    double lastSeconds = 0;

    /**
     *
     * @param error The error; Zero is the perfect one.
     * @return
     */
    public double update(double error) {
        double nowSeconds = elapsedTime.seconds();
        double adjustment = coefficients.p * error +
                coefficients.i * iSum +
                coefficients.d * (error - lastError)/(nowSeconds - lastSeconds) +
                (Math.signum(error) == -1 ? coefficients.f : 0);
        iSum += (nowSeconds - lastSeconds) * error;

        lastSeconds = nowSeconds;
        lastError = error;
        return adjustment;
    }
    public PIDFController(PIDFCoefficients coefficients) {
        this.coefficients = coefficients;
    }
    public PIDFController(double p, double i, double d, double f) {
        this.coefficients = new PIDFCoefficients(p, i, d, f);
    }
}
