package org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDFController;

@Config
public class HorizontalSlides {
    public static boolean outputDebugInfo = false;

    final static int MIN = -450;
    final static int MAX = 0;

    public DcMotorEx horizontalSlidesMotor;
    public static PIDFCoefficients coefficients = new PIDFCoefficients(-0.015,0,-0,0);
    public PIDFController pidController = new PIDFController(coefficients);
    public enum State {
        userControlled,
        runToPosition
    }
    private double position = 0;
    private State state = State.userControlled;

    public HorizontalSlides (HardwareMap hardwareMap, boolean resetEncoders) {
        horizontalSlidesMotor = hardwareMap.get(DcMotorEx.class,"horizontalSlidesMotor");
        if(resetEncoders) {
            horizontalSlidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        horizontalSlidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalSlidesMotor.setDirection(DcMotor.Direction.REVERSE);
        horizontalSlidesMotor.setCurrentAlert(7, CurrentUnit.AMPS);

    }
    public HorizontalSlides (HardwareMap hardwareMap) {
        this(hardwareMap, true);
    }
    public void setPosition(double position, boolean changeState) {
        if(changeState) state = State.runToPosition;
        this.position = position;
    }
    public void setPosition(double position) {
        setPosition(position, true);
    }

    public void setPower(double power) {
        state = State.userControlled;
        horizontalSlidesMotor.setPower(-power);
    }

    //set power with limits
    public void trySetPower(double power) {
        int slidePos = horizontalSlidesMotor.getCurrentPosition();
        if ((slidePos>=MIN && power >= 0) || (slidePos <= MAX && power <= 0)) {
            setPower(power);
        } else {
            setPower(0);
        }
    }

    public boolean resetSlides() {
        if (!horizontalSlidesMotor.isOverCurrent()) {
            horizontalSlidesMotor.setPower(1);
            return false;
        } else {
            horizontalSlidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            horizontalSlidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            return true;
        }
    }

    public int getPosition() {
        return horizontalSlidesMotor.getCurrentPosition();
    }

    public double getTargetPosition() {
        return position;
    }
    public State getState() {
        return state;
    }

    public void update() {
        if (state == State.runToPosition) {
            horizontalSlidesMotor.setPower(pidController.update(horizontalSlidesMotor.getCurrentPosition() - position));
            // Brake if already in position
            if (Math.abs(horizontalSlidesMotor.getCurrentPosition() - position) < 5) {
                state = State.userControlled;
                setPower(0);
            }
        }
        if(outputDebugInfo) {
            FtcDashboard dashboard = FtcDashboard.getInstance();
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("HorizontalSlides/Current", horizontalSlidesMotor.getCurrentPosition());
            packet.put("HorizontalSlides/Target", position);
            packet.put("HorizontalSlides/State", state);
            packet.put("HorizontalSlides/CurrentLOad", horizontalSlidesMotor.getCurrent(CurrentUnit.AMPS));
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
