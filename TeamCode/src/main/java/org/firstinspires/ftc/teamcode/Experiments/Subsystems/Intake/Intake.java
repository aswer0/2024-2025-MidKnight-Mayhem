package org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    public DcMotorEx intakeMotor;

    public Servo intakePivot;

    public RevColorSensorV3 intakeSensor;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class,"intakeMotor");
        intakePivot = hardwareMap.get(Servo.class,"intakePivot");
        intakeSensor = hardwareMap.get(RevColorSensorV3.class,"intakeSensor");
    }

}
