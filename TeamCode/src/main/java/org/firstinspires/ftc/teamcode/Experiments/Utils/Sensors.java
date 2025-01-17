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
    DistanceSensor backDistanceSensor;

    RevColorSensorV3 intakeSensor;
    public static double chamberThreshold = 5.5;
    public static double ArmDistchamberThreshold = 6;
    Double[] distSensorReads = {0.0, 0.0, 0.0, 0.0, 0.0};
    int updateCounter=0;

    public Sensors(HardwareMap hardwareMap, Telemetry telemetry)  {
        frontDistanceSensor = hardwareMap.get(DistanceSensor.class, "fDS");

        //need new config
        backDistanceSensor = hardwareMap.get(DistanceSensor.class, "bDS");
        intakeSensor = hardwareMap.get(RevColorSensorV3.class, "iS");
        this.telemetry = telemetry;

    }

    public void update() {
        distSensorReads[updateCounter] = get_front_dist();
        updateCounter++;
    }

    public boolean atChamber() {
        return get_front_dist() < chamberThreshold;
    }
    public boolean atArmDistChamber() {
        return get_front_dist() < ArmDistchamberThreshold;
    }

    public double get_front_dist(){
        return frontDistanceSensor.getDistance(DistanceUnit.INCH);
    }
    public double get_back_dist(){
        return backDistanceSensor.getDistance(DistanceUnit.INCH);
    }

    public double get_average_front_dist(){
        double sum = 0;
        for (int i=0; i<5; i++) sum+= distSensorReads[i];
        return sum/5;
    }

    public boolean sampleIn() {
        return intakeSensor.getDistance(DistanceUnit.CM) <2.5;
    }

    public int getIntakeColor() {
        int redValue = intakeSensor.red();
        int greenValue = intakeSensor.green();
        int blueValue = intakeSensor.blue();
        //if(true) return ((double)redValue + greenValue + blueValue)/3 > 125 ? 2 : 0;

        if (blueValue > 450) { //blue
            return 3;
        } else if (blueValue > 200 && redValue > 300 && 250 < greenValue && greenValue < 500) { //yellow
            return 2;
        } else if (blueValue > 300 && redValue > 400 && greenValue < 600) { //blue
            return 3;
        } else { //none
            return 0;
        }
    }
}
