package org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Manipulator {
    public Servo pitch1;
    public Servo pitch2;
    public Servo flap;

    public DistanceSensor outtakeSensor;

    ElapsedTime timer;

    public Manipulator(HardwareMap hardwareMap) {
        this.pitch1 = hardwareMap.get(Servo.class, "pitch1");
        this.pitch2 = hardwareMap.get(Servo.class, "pitch2");
        this.flap = hardwareMap.get(Servo.class, "flap");

        outtakeSensor = hardwareMap.get(DistanceSensor.class, "outtakeSensor");

        timer = new ElapsedTime();
    }
}
