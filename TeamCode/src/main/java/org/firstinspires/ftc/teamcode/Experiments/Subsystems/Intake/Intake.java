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
    public boolean hasCorrectObject = false;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(CRServo.class,"intakeMotor");
        intakePivot = hardwareMap.get(Servo.class,"intakePivot");
        intakeSensor = hardwareMap.get(RevColorSensorV3.class,"intakeSensor");
    }
    public void setPivot(double pos) {
        intakePivot.setPosition(pos);
    }
    public void update(boolean posessingObject) {
        hasCorrectObject = intakeSensor.getDistance(DistanceUnit.INCH) < 0.1;
        // TODO: Alliance detection
        if(intaking && hasCorrectObject) {
            intakeMotor.setPower(0);
        } else if(intaking) {
            intakeMotor.setPower(1);
        } else if (!hasCorrectObject) { // bad object or no object, reverse both. TODO maybe make a distinction between both objects
            intakeMotor.setPower(-1);
        } else {
            intakeMotor.setPower(0);
        }
    }
    public void update() {
        update(false);
    }
}
