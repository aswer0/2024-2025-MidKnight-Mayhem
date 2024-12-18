package org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Experiments.Utils.Alliance;

@Config
public class Intake {
    public static boolean outputDebugInfo = false;

    public static double DOWN_POS=0.47;
    public static double UP_POS=0.9;

    public CRServo intakeServo1;
    public CRServo intakeServo2;

    public Servo intakePivotLeft;
    public Servo intakePivotRight;

    public RevColorSensorV3 intakeSensor;
    public boolean intaking = false;
    public boolean hasCorrectObject = false;
    public Alliance alliance = Alliance.red;

    public Intake(HardwareMap hardwareMap) {
        intakeServo1 = hardwareMap.get(CRServo.class,"iS1"); //left looking from intake side
        intakeServo2 = hardwareMap.get(CRServo.class,"iS2");
        intakePivotLeft = hardwareMap.get(Servo.class,"iPL"); //from intake side
        intakePivotRight = hardwareMap.get(Servo.class,"iPR");
        intakeSensor = hardwareMap.get(RevColorSensorV3.class,"iS");
        intakeSensor.enableLed(false);
    }
    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }
    public void setPower(double power) {
        intakeServo1.setPower(power);
        intakeServo2.setPower(-power);
    }

    public void intake() {
        setPower(-1);
    }
    public void reverse() {
        setPower(1);
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
        hasCorrectObject = ((alliance == Alliance.red ? intakeSensor.getNormalizedColors().red > 120 : intakeSensor.getNormalizedColors().blue > 80) || (intakeSensor.getNormalizedColors().red > 120 && intakeSensor.getNormalizedColors().green > 120)) && intakeSensor.getDistance(DistanceUnit.INCH) < 2.5;
        // TODO: Alliance detection
        if(intaking && hasCorrectObject) {
            //stop();
        } else if(intaking) {
            //intake();
        } else if (!hasCorrectObject) { // bad object or no object, reverse both. TODO maybe make a distinction between both objects
            //reverse();
        } else {
            //stop();
        }
        if(outputDebugInfo) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Intake/Red", intakeSensor.red() );
            packet.put("Intake/Green", intakeSensor.green());
            packet.put("Intake/Blue", intakeSensor.blue());
            packet.put("Intake/Distance", intakeSensor.getDistance(DistanceUnit.INCH));
            packet.put("Intake/hasCorrectObject", hasCorrectObject);
            (FtcDashboard.getInstance()).sendTelemetryPacket(packet);
        }
    }
    public void update() {
        update(false);
    }
}
