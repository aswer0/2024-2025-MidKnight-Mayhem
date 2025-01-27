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

@Config
public class Intake {
    public static boolean outputDebugInfo = false;

    public static double DOWN_POS=0.485;
    public static double UP_POS=0.9;

    public static double DOOR_OPEN_POS=0.5;
    public static double DOOR_CLOSE_POS=0.5;

    public DcMotorEx intakeMotor;

    public Servo intakePivotLeft;
    public Servo intakePivotRight;

    //public Servo intakeDoor;

    public Servo sweeper;

    public RevColorSensorV3 intakeSensor;
    public boolean intaking = false;
    public boolean hasCorrectObject = false;
    public Alliance alliance = Alliance.red;
    public Sensors sensors;

    public Intake(HardwareMap hardwareMap, Sensors sensors) {
        this.sensors = sensors;
        intakeMotor = hardwareMap.get(DcMotorEx.class,"intakeMotor"); //left looking from intake side
        intakePivotLeft = hardwareMap.get(Servo.class,"iPL"); //from intake side
        intakePivotRight = hardwareMap.get(Servo.class,"iPR");
        //intakeDoor = hardwareMap.get(Servo.class, "intakeDoor");
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
        setPower(1);
    }
    public void reverse() {
        setPower(-0.75);
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
    public void reverseDown() {setPivot(0.55);}

    //public void openDoor() {intakeDoor.setPosition(DOOR_OPEN_POS);}
    //public void closeDoor() {intakeDoor.setPosition(DOOR_CLOSE_POS);}

    public void sweeperIn() {sweeper.setPosition(0);}
    public void sweeperOut() {sweeper.setPosition(0.5);}

    public boolean smartIntake() {
        int intakeColor = sensors.getIntakeColor();
        if (alliance==Alliance.red) {
            return intakeColor == 1 || intakeColor == 2;
//            if (intakeColor == 1 || intakeColor == 2) {
//                return true;
//            } else {
//
//            }
        } else {
            return intakeColor == 3 || intakeColor == 2;
        }
    }

    public void update(boolean posessingObject) {
        //hasCorrectObject = ((alliance == Alliance.red ? intakeSensor.getNormalizedColors().red > 120 : intakeSensor.getNormalizedColors().blue > 80) || (intakeSensor.getNormalizedColors().red > 120 && intakeSensor.getNormalizedColors().green > 120)) && intakeSensor.getDistance(DistanceUnit.INCH) < 2.5;
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
