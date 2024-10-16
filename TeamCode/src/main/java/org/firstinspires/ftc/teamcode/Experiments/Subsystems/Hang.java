package org.firstinspires.ftc.teamcode.Experiments.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hang {
    Servo hangLeft;
    //Servo hangRight; (unused)

    public Hang(HardwareMap hardwareMap) {
        hangLeft = hardwareMap.get(Servo.class,"hangLeft");
        //hangRight = hardwareMap.get(Servo.class,"hangRight");
    }
}
