package org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Arm {
    Servo oAF;
    Servo oAN;
    Servo oPL;
    Servo oPR;
    Servo tClaw;

    public static Preset outtakeSpecimen = new Preset(0.9,0.1,0.38, 0.5);
    public static Preset outtakeSample = new Preset(0,0,0.5, 0.5);
    public static Preset intakeSpecimen = new Preset(0,0,0.5, 0.5);
    public static Preset intakeSample = new Preset(0,0,0.5, 0.5);


    public Arm(HardwareMap hardwareMap) {
        this.oAF = hardwareMap.get(Servo.class, "oAF");
        this.oAN = hardwareMap.get(Servo.class, "oAN");
        this.oPL = hardwareMap.get(Servo.class, "oPL");
        this.oPR = hardwareMap.get(Servo.class, "oPR");
        this.tClaw = hardwareMap.get(Servo.class, "tClaw");
    }
    public void applyPreset(Preset preset) {
        oAF.setPosition(preset.oAF);
        oAN.setPosition(preset.oAN);
        oPL.setPosition(preset.oP);
        oPR.setPosition(1-preset.oP);
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
        public double oAF;
        public double oAN;
        public double oP;
        public double tClaw;
        public Preset(double oAF, double oAN, double oP, double tClaw) {
            this.oAF = oAF;
            this.oAN = oAN;
            this.oP = oP;
            this.tClaw = tClaw;
        }
    }
}
