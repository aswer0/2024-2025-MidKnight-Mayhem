package org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Intake {
    public CRServo intakeMotor;

    public Servo intakePivot;

    public RevColorSensorV3 intakeSensor;
    public boolean intaking = false;
    public boolean hasObject = false;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(CRServo.class,"intakeMotor");
        intakePivot = hardwareMap.get(Servo.class,"intakePivot");
        intakeSensor = hardwareMap.get(RevColorSensorV3.class,"intakeSensor");
    }

    public void update(boolean posessingObject) {
        hasObject = intakeSensor.getDistance(DistanceUnit.INCH) < 0.1;
        // TODO: Alliance detection
        if(posessingObject) {
            intakeMotor.setPower(-1);
        } else if(intaking && !hasObject) {
            intakeMotor.setPower(1);
        } else {
            intakeMotor.setPower(0);
        }
    }
}
