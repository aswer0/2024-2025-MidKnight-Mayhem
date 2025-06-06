package org.firstinspires.ftc.teamcode.FinalPrograms.Subsystems.Outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Manipulator {
    public static double claw_pos_close = 0.43; //0.28
    public static double claw_pos_open = 0.15; //0.52

    public Servo armPitch;
    public Servo bucketPitch;
    public Servo flap;
    public Servo claw;

    public DistanceSensor outtakeDistanceSensor;
    public RevColorSensorV3 clawColorSensor;

    int redValue, greenValue, blueValue;

    ElapsedTime timer;

    public Manipulator(HardwareMap hardwareMap) {
       // this.armPitch = hardwareMap.get(Servo.class, "armPitch");
        //this.bucketPitch = hardwareMap.get(Servo.class, "bucketPitch");
        //this.flap = hardwareMap.get(Servo.class, "flap");
        this.claw = hardwareMap.get(Servo.class, "claw");

        //outtakeDistanceSensor = hardwareMap.get(DistanceSensor.class, "outtakeDistanceSensor");
        //clawColorSensor = hardwareMap.get(RevColorSensorV3.class, "clawColorSensor");

        timer = new ElapsedTime();
    }

    public void openClaw() {
        claw.setPosition(claw_pos_open);
    }
    public void closeClaw() {
        claw.setPosition(claw_pos_close);
    }

    /**
     *
     * @return Whether I have the sample
     */
    public boolean clawHasObject() {
        redValue = clawColorSensor.red();
        greenValue = clawColorSensor.green();
        blueValue = clawColorSensor.blue();
        return (redValue >= 609 || blueValue >= 609)
                && ((redValue >= blueValue && redValue >= greenValue)
                    || (blueValue >= redValue && blueValue >= greenValue));
    }

}
