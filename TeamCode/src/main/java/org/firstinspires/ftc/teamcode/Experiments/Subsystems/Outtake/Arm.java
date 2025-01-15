package org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Arm {
    Servo outtake_arm_far;
    Servo outtake_arm_near ;
    Servo pivot_left;
    Servo pivot_right;
    Servo tClaw;

    public static Preset outtakeSpecimen = new Preset(0.9,0.1,0.38, 0.5);
    public static Preset outtakeSample = new Preset(0,0,0.5, 0.5);
    public static Preset intakeSpecimen = new Preset(0,0,0.5, 0.5);
    public static Preset intakeSample = new Preset(0,0,0.5, 0.5);


    public Arm(HardwareMap hardwareMap) {
        this.outtake_arm_far = hardwareMap.get(Servo.class, "outtake_arm_far");
        this.outtake_arm_near  = hardwareMap.get(Servo.class, "outtake_arm_near ");
        this.pivot_left = hardwareMap.get(Servo.class, "pivot_left");
        this.pivot_right = hardwareMap.get(Servo.class, "pivot_right");
        this.tClaw = hardwareMap.get(Servo.class, "tClaw");
    }
    public void applyPreset(Preset preset) {
        outtake_arm_far.setPosition(preset.outtake_arm_far);
        outtake_arm_near .setPosition(preset.outtake_arm_near );
        pivot_left.setPosition(preset.oP);
        pivot_right.setPosition(1-preset.oP);
        tClaw.setPosition(preset.tClaw);
    }
    public void outtakeSpecimen() {
        applyPreset(outtakeSpecimen);
    }
    public void outtakeSample() {
        applyPreset(outtakeSample);
    }
    public void intakeSpecimen() {
        applyPreset(intakeSpecimen);
    }
    public void intakeSample() {
        applyPreset(intakeSample);
    }
    public static class Preset {
        public double outtake_arm_far;
        public double outtake_arm_near ;
        public double oP;
        public double tClaw;
        public Preset(double outtake_arm_far, double outtake_arm_near , double oP, double tClaw) {
            this.outtake_arm_far = outtake_arm_far;
            this.outtake_arm_near  = outtake_arm_near ;
            this.oP = oP;
            this.tClaw = tClaw;
        }
    }
}
