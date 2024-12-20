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
    //ArrayList<Double> distSensorReads;

    public Sensors(HardwareMap hardwareMap, Telemetry telemetry)  {
       // distSensorReads.add(0.0);
        //distSensorReads.clear();
        //for (int i=0; i<5; i++) distSensorReads.add(999.0);
        frontDistanceSensor = hardwareMap.get(DistanceSensor.class, "fDS");
        iS = hardwareMap.get(RevColorSensorV3.class, "iS");
        this.telemetry = telemetry;

    }

    //public void update() {
        //distSensorReads.add(0, get_front_dist());
    //}

    public boolean atChamber() {
        return get_front_dist() < chamberThreshold;
    }

    public double get_front_dist(){
        return frontDistanceSensor.getDistance(DistanceUnit.INCH);
    }

    //public double get_average_front_dist(){
    //    double sum = 0;
    //    for (int i=0; i<5; i++) sum+= distSensorReads.get(i);
    //    return sum/5;
    //}

    public boolean sampleIn() {
        return iS.getDistance(DistanceUnit.CM) <2.5;
    }
}
