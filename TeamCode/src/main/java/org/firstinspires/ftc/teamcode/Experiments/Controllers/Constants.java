package org.firstinspires.ftc.teamcode.Experiments.Controllers;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class Constants {
    public static double f = 0.06;
    public static double secondary_dist = 3;
    public static double power_to_velocity = 65;

    public static boolean reset_i = true;
    public static PIDCoefficients primary_drive_coeff = new PIDCoefficients(0.1, 0, 0.01);
    public static PIDCoefficients secondary_drive_coeff = new PIDCoefficients(0.1, 0.1, 0.01);

    public static PIDCoefficients primary_translational_coeff = new PIDCoefficients(0.1, 0, 0.002);
    public static PIDCoefficients secondary_translational_coeff = new PIDCoefficients(0.2, 0.05, 0.001);

    public static PIDCoefficients primary_heading_coeff = new PIDCoefficients(0.03, 0, 0.002);
    public static PIDCoefficients secondary_heading_coeff = new PIDCoefficients(0.03, 0.025, 0.002);

    // Default controllers
    public static ScheduledPID drive_controller = new ScheduledPID(
            primary_drive_coeff,
            secondary_drive_coeff,
            "Drive",
            secondary_dist,
            reset_i);

    public static ScheduledPID translational_controller = new ScheduledPID(
            primary_translational_coeff,
            secondary_translational_coeff,
            "Drive",
            secondary_dist,
            reset_i);

    public static ScheduledPID heading_controller = new ScheduledPID(
            primary_heading_coeff,
            secondary_heading_coeff,
            "Heading",
            secondary_dist,
            reset_i);
}
