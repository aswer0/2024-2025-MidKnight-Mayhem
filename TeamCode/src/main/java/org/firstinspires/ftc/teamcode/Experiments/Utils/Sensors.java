package org.firstinspires.ftc.teamcode.Experiments.Utils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Sensors {
    Telemetry telemetry;
    DistanceSensor frontDistanceSensor;
    public static double chamberThreshold = 5.5;

    public Sensors(HardwareMap hardwareMap, Telemetry telemetry)  {
        frontDistanceSensor = hardwareMap.get(DistanceSensor.class, "fDS");
        this.telemetry = telemetry;

    }
    public boolean atChamber() {
        return frontDistanceSensor.getDistance(DistanceUnit.INCH) < chamberThreshold;
    }

    public double get_front_dist(){
        return frontDistanceSensor.getDistance(DistanceUnit.INCH);
    }
}
