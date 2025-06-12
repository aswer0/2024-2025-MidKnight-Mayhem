package org.firstinspires.ftc.teamcode.FinalPrograms.Subsystems.Outtake;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDFController;

@Config
public class Lift {
    public static boolean outputDebugInfo = false;

    public static int MIN = 0;
    public static int MAX = 2500;

    public static double highBasketPos=2100;
    public static double highChamberPos=150;
    public static double lowBasketPos=90;
    public static double lowChamberPos=0;
    public static double intakeSpecimenPos=25;
    public static double intakeSamplePos=0;
    public static double backChamber1Pos=450;
    public static double backChamber2Pos=1000;
    public static double hangHigh=2450;
    public static double hangLow=1400;

    public static double oldSpecIntake=200;
    public static double oldSpecOuttake=1500;


    public DcMotorEx leftSlide;
    public DcMotorEx rightSlide;

    public static PIDFCoefficients coefficients = new PIDFCoefficients(-0.005,0, 0, 0.1);

    public PIDFController motorController = new PIDFController(coefficients);;
    public double cap = Double.POSITIVE_INFINITY;

    public enum State {
        userControlled,
        runToPosition
    }
    private double position = 0;
    private State state = State.userControlled;

    public Lift (HardwareMap hardwareMap, boolean resetEncoders) {
        this.leftSlide = hardwareMap.get(DcMotorEx.class,"SlideLeft"); //The one with the encoder
        if(resetEncoders)this.leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.leftSlide.setCurrentAlert(5, CurrentUnit.AMPS);

        this.rightSlide = hardwareMap.get(DcMotorEx.class,"SlideRight");
        this.rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public Lift (HardwareMap hardwareMap) {
        this(hardwareMap, true);
    }

    public void brakeSlides(boolean brake){
        if(brake){
            this.leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }else{
            this.leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            this.rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public void toHighBasket() {
        setPosition(highBasketPos);
    }

    public void toHighChamber() {
        setPosition(highChamberPos);
    }

    public void toLowBasket() {
        setPosition(lowBasketPos);
    }

    public void toLowChamber() {
        setPosition(lowChamberPos);
    }
    public void intakeSpecimen() {
        setPosition(intakeSpecimenPos);
    }
    public void intakeSample() {
        setPosition(intakeSamplePos);
    }
    public void toBackChamber1Pos() {setPosition(backChamber1Pos);}
    public void toBackChamber2Pos() {setPosition(backChamber2Pos);}

    public void toOldSpecIntake() {setPosition(oldSpecIntake);}
    public void toOldSpecOuttake() {setPosition(oldSpecOuttake);}

    public void setPosition(double position, boolean changeState) {
        setPosition(position, changeState, Double.POSITIVE_INFINITY);
    }
    public void setPosition(double position, boolean changeState, double cap) {
        if(changeState) state = State.runToPosition;
        this.cap = cap;
        this.position = position;
    }
    public void setPosition(double position) {
        setPosition(position, true);
    }
    public void setPosition(double position, double cap) {
        setPosition(position, true, cap);
    }

    public void setPower(double power) {
        state = State.userControlled;
        leftSlide.setPower(power);
        rightSlide.setPower(power);
    }

    public void trySetPower(double power) {
        int slidePos = leftSlide.getCurrentPosition();
        if ((slidePos>=MIN && power <= 0) || (slidePos <= MAX && power >= 0)) {
            setPower(power);
        } else {
            setPower(0);
        }
    }

    public State getState() {
        return state;
    }
    public double getPosition() {
        return position;
    }
    public double getCurrentPos() {
        return leftSlide.getCurrentPosition();
    }
    public boolean resetSlides() {
        if (!leftSlide.isOverCurrent()) {
            leftSlide.setPower(-1);
            rightSlide.setPower(-1);
            return false;
        } else {
            leftSlide.setPower(0);
            rightSlide.setPower(0);
            leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            return true;
        }
    }
    public void update() {
        if (state == State.runToPosition) {
            double input = Math.min(motorController.update(getCurrentPos() - position), cap);
            leftSlide.setPower(input);
            rightSlide.setPower(input);
            if(Math.abs(leftSlide.getCurrentPosition() - position) < 5 && position == 0) {
                state = State.userControlled;

                setPower(0);
            }
        }
        if(outputDebugInfo) {
            FtcDashboard dashboard = FtcDashboard.getInstance();
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Lift/Current", leftSlide.getCurrentPosition());
            packet.put("Lift/Target", position);
            packet.put("Lift/State", state);
            packet.put("Lift/Power", leftSlide.getPower());
            dashboard.sendTelemetryPacket(packet);
        }
    }

}
