package org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Experiments.Utils.Alliance;
import org.firstinspires.ftc.teamcode.Experiments.Utils.Sensors;

import java.util.Objects;

@Config
public class Intake {
    public static boolean outputDebugInfo = false;

    public static double DOWN_POS=0.64;
    public static double UP_POS=0.9;

    public static double DOOR_OPEN_POS=0.8;
    public static double DOOR_CLOSE_POS=0.4;

    public DcMotorEx intakeMotor;

    public Servo intakePivotLeft;
    public Servo intakePivotRight;

    public Servo intakeDoor;

    public Servo sweeper;

    public RevColorSensorV3 intakeSensor;
    public Alliance alliance = Alliance.red;
    public Sensors sensors;

    public Intake(HardwareMap hardwareMap, Sensors sensors) {
        this.sensors = sensors;
        intakeMotor = hardwareMap.get(DcMotorEx.class,"intakeMotor"); //left looking from intake side
        intakePivotLeft = hardwareMap.get(Servo.class,"iPL"); //from intake side
        intakePivotRight = hardwareMap.get(Servo.class,"iPR");
        intakeDoor = hardwareMap.get(Servo.class, "intakeDoor");
        sweeper = hardwareMap.get(Servo.class, "sweeper");
        intakeSensor = hardwareMap.get(RevColorSensorV3.class,"iS");
        intakeSensor.enableLed(false);
    }
    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }
    public void setPower(double power) {
        intakeMotor.setPower(power);
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
    public void reverseDown() {setPivot(0.78);}

    public void openDoor() {intakeDoor.setPosition(DOOR_OPEN_POS);}
    public void closeDoor() {intakeDoor.setPosition(DOOR_CLOSE_POS);}

    public void sweeperIn() {sweeper.setPosition(0);}
    public void sweeperOut() {sweeper.setPosition(0.5);}


    public boolean smartIntake(boolean detectYellow) {
        SampleColor color = getColor();
        if(color == SampleColor.yellow && !detectYellow) color = SampleColor.wrong; // do not intake yellows if not told to; this feels hacky
        if(outputDebugInfo) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Intake/color", color);
            (FtcDashboard.getInstance()).sendTelemetryPacket(packet);
        }
        switch(color) {

            case allianceSpecific:
                intake();
                closeDoor();
                return true;
            case wrong:
                intake();
                openDoor();
                return false;
            case unsure:
            case none:
                intake();
                closeDoor();
                return false;
        }
        return false;
    }
    public boolean hasCorrectSample(boolean detectYellow) {
        SampleColor color = getColor();
        return color == SampleColor.allianceSpecific || (detectYellow && color == SampleColor.yellow);
    }
    public SampleColor getColor(){
        if(intakeSensor.getDistance(DistanceUnit.INCH) > 2) return SampleColor.none;
        boolean red = 50 < intakeSensor.red() && intakeSensor.red() < 80 &&
                50 < intakeSensor.blue() && intakeSensor.blue() < 70 &&
                60 < intakeSensor.green() && intakeSensor.green() < 80;
        boolean blue = 17 < intakeSensor.red() && intakeSensor.red() < 43 &&
                45 < intakeSensor.green() && intakeSensor.green() < 77 &&
                85 < intakeSensor.blue() && intakeSensor.blue() < 120;
        boolean yellow = (73 < intakeSensor.red() && intakeSensor.red() < 110 &&
                114 < intakeSensor.green() && intakeSensor.green() < 160 &&
                55 < intakeSensor.blue() && intakeSensor.blue() < 80);
        if(outputDebugInfo) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Intake/Red", intakeSensor.red());
            packet.put("Intake/Green", intakeSensor.green());
            packet.put("Intake/Blue", intakeSensor.blue());
            packet.put("Intake/Distance", intakeSensor.getDistance(DistanceUnit.INCH));
            packet.put("Intake/hasRed", red);
            packet.put("Intake/hasBlue", blue);
            packet.put("Intake/hasYellow", yellow);
            (FtcDashboard.getInstance()).sendTelemetryPacket(packet);
        }
        // See a truth table at ( https://truth-table.com/#(r%20%7C%7C%20b%20%7C%7C%20!y)%20&&%20(r%20%7C%7C%20!b%20%7C%7C%20y)%20&&%20(!r%20%7C%7C%20b%20%7C%7C%20y) )
        if((red || blue || !yellow)
                && (red  || !blue || yellow)
                && (!red || blue  || yellow)) return SampleColor.unsure; // the if statement is false if exactly one of the values is true, true otherwise.
        if(yellow) return SampleColor.yellow;

        if(alliance == Alliance.red) {
            return red ? SampleColor.allianceSpecific : SampleColor.wrong;
        } else {
            return blue ? SampleColor.allianceSpecific : SampleColor.wrong; // the alliance must be blue
        }
    }
    public enum SampleColor {
        /** Red color if alliance is red, blue if alliance is blue */
        allianceSpecific,
        /** Yellow, regardless of alliance */
        yellow,
        /** Blue if alliance color is red, red if alliance color is blue */
        wrong,
        /** The color is unknown, probably if the color sensors are mistuned */
        unsure,
        /** There is no object in the intake */
        none
    }
}
