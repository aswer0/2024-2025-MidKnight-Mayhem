package org.firstinspires.ftc.teamcode.Experiments.Utils;

public class utils {
    public static double mod(double n, double m){
        return (m + n % m) % m;
    }

    public static double cap(double val, double low, double high){
        return low + mod(val-low, high-low+1);
    }

    public static double map(double val, double low, double high, double min, double max){
        return (val - low) * (max - min) / (high - low) + min;
    }
}
