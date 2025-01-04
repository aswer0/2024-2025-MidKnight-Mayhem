package org.firstinspires.ftc.teamcode.Experiments.Utils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;

@Config
public class Sensors {
    Telemetry telemetry;
    DistanceSensor frontDistanceSensor;
    RevColorSensorV3 iS;
    public static double chamberThreshold = 5.5;
    Double[] distSensorReads = {0.0, 0.0, 0.0, 0.0, 0.0};
    int updateCounter=0;

    public Sensors(HardwareMap hardwareMap, Telemetry telemetry)  {
        frontDistanceSensor = hardwareMap.get(DistanceSensor.class, "fDS");
        iS = hardwareMap.get(RevColorSensorV3.class, "iS");
        this.telemetry = telemetry;

    }

    public void update() {
        distSensorReads[updateCounter] = get_front_dist();
        updateCounter++;
    }

    public boolean atChamber() {
        return get_front_dist() < chamberThreshold;
    }

    public double get_front_dist(){
        return frontDistanceSensor.getDistance(DistanceUnit.INCH);
    }

    public double get_average_front_dist(){
        double sum = 0;
        for (int i=0; i<5; i++) sum+= distSensorReads[i];
        return sum/5;
    }

    public boolean sampleIn() {
        return iS.getDistance(DistanceUnit.CM) <2.5;
    }
}
