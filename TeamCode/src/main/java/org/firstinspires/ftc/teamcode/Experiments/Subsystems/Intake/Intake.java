package org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Intake {
    double DOWN_POS=0.2;
    double UP_POS=0.9;

    public CRServo intakeServo1;
    public CRServo intakeServo2;

    public Servo intakePivotLeft;
    public Servo intakePivotRight;

    public RevColorSensorV3 intakeSensor;
    public boolean intaking = false;
    public boolean hasCorrectObject = false;

    public Intake(HardwareMap hardwareMap) {
        intakeServo1 = hardwareMap.get(CRServo.class,"intakeServo1"); //left looking from intake side
        intakeServo2 = hardwareMap.get(CRServo.class,"intakeServo2");
        intakePivotLeft = hardwareMap.get(Servo.class,"intakePivotLeft"); //from intake side
        intakePivotRight = hardwareMap.get(Servo.class,"intakePivotRight");
        intakeSensor = hardwareMap.get(RevColorSensorV3.class,"intakeSensor");
    }

    public void setPower(double power) {
        intakeServo1.setPower(power);
        intakeServo2.setPower(-power);
    }

    public void intake() {
        setPower(1);
    }
    public void reverse() {
        setPower(-1);
    }
    public void stop() {
        setPower(0);
    }

    public void setPivot(double pos) {
        intakePivotLeft.setPosition(pos);
        intakePivotRight.setPosition(1-pos);
    }
    public void up() {setPivot(UP_POS);}
    public void down() {setPivot(DOWN_POS);}

    public void update(boolean posessingObject) {
        hasCorrectObject = intakeSensor.getDistance(DistanceUnit.INCH) < 0.1;
        // TODO: Alliance detection
        if(intaking && hasCorrectObject) {
            stop();
        } else if(intaking) {
            intake();
        } else if (!hasCorrectObject) { // bad object or no object, reverse both. TODO maybe make a distinction between both objects
            reverse();
        } else {
            stop();
        }
    }
    public void update() {
        update(false);
    }
}
