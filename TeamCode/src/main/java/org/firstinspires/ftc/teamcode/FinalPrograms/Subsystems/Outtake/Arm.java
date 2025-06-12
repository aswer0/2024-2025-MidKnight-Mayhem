package org.firstinspires.ftc.teamcode.FinalPrograms.Subsystems.Outtake;

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

    public static Preset outtakeSpecimen1 = new Preset(0.1,0.5,0.28);
    public static Preset outtakeSpecimen2 = new Preset(0.1,0.1,0.28);
    public static Preset outtakeSample = new Preset(0.5,0.93,0.35);
    public static Preset intakeSpecimen = new Preset(0.85,0.56,0.73);
    public static Preset intakeSample = new Preset(0.09,0.25,0.74);
    public static Preset idlePosition = new Preset(0.1,0.215,0.67);
    public static Preset specIdlePosition = new Preset(0.85,0.205,0.69);
    public static Preset autoStartPosition = new Preset(0.5,0.93,0.72);
    public static Preset teleStartPosition = new Preset(0.1,0.5,0.42);
    public static Preset backOuttakeSpecimen1 = new Preset(0.1,0.49,0.64);
    public static Preset backOuttakeSpecimen2 = new Preset(0.1,0.49,0.56);

    public static double clawClosePos = 0.335; //0.385
    public static double clawHalfOpenPos = 0.34;
    public static double clawOpenPos = 0.6; //0.15

    public Arm(HardwareMap hardwareMap) {
        this.outtake_arm_far = hardwareMap.get(Servo.class, "oAF");
        this.outtake_arm_near  = hardwareMap.get(Servo.class, "oAN");
        this.pivot_left = hardwareMap.get(Servo.class, "oPL");
        this.pivot_right = hardwareMap.get(Servo.class, "oPR");
        this.tClaw = hardwareMap.get(Servo.class, "tClaw");
    }
    public void applyPreset(Preset preset) {
        outtake_arm_far.setPosition(preset.outtake_arm_far);
        outtake_arm_near.setPosition(preset.outtake_arm_near);
        pivot_left.setPosition(preset.oP);
        pivot_right.setPosition(1-preset.oP);
    }
    public void outtakeSpecimen1() {
        applyPreset(outtakeSpecimen1);
    }
    public void outtakeSpecimen2() {
        applyPreset(outtakeSpecimen2);
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
    public void toIdlePosition() {
        applyPreset(idlePosition);
    }
    public void toSpecIdlePosition() {
        applyPreset(specIdlePosition);
    }
    public void toAutoStartPosition() {
        applyPreset(autoStartPosition);
    }
    public void toTeleStartPosition() {
        applyPreset(teleStartPosition);
    }
    public void backOuttakeSpecimen1() {
        applyPreset(backOuttakeSpecimen1);
    }
    public void backOuttakeSpecimen2() {
        applyPreset(backOuttakeSpecimen2);
    }

    public void openClaw() {
        tClaw.setPosition(clawOpenPos);
    }
    public void halfOpenClaw() {
        tClaw.setPosition(clawHalfOpenPos);
    }
    public void closeClaw() {
        tClaw.setPosition(clawClosePos);
    }

    public double getNearPosition() {
        return outtake_arm_near.getPosition();
    }
    public double getFarPosition() {
        return outtake_arm_far.getPosition();
    }
    public double getPivotPosition() {
        return pivot_left.getPosition();
    }

    public static class Preset {
        public double outtake_arm_far;
        public double outtake_arm_near ;
        public double oP;
        //public double tClaw;
        public Preset(double outtake_arm_far, double outtake_arm_near , double oP) { //double tClaw
            this.outtake_arm_far = outtake_arm_far;
            this.outtake_arm_near  = outtake_arm_near ;
            this.oP = oP;
            //this.tClaw = tClaw;
        }
    }
}
