package org.firstinspires.ftc.teamcode.Experiments.Utils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Sensors {
    DistanceSensor frontDistanceSensor;
    public static double chamberThreshold = 1;

    public Sensors(HardwareMap hardwareMap)  {
        frontDistanceSensor = hardwareMap.get(DistanceSensor.class, "frontDistanceSensor");
    }
    public boolean atChamber() {
        return frontDistanceSensor.getDistance(DistanceUnit.INCH) < chamberThreshold;
    }
}
