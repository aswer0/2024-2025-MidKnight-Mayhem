package org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDController;
import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDFController;

import java.util.concurrent.CompletableFuture;

@Config
public class HorizontalSlides {
    public static boolean outputDebugInfo = false;

    final static int MIN = -490;
    final static int MAX = 0;

    public DcMotorEx horizontalSlidesMotor;
    public static PIDFCoefficients coefficients = new PIDFCoefficients(-0.055,0,-0.0005,0);
    public PIDFController pidController = new PIDFController(coefficients);
    public enum State {
        userControlled,
        runToPosition
    }
    private double position = 0;
    private State state = State.userControlled;

    public HorizontalSlides (HardwareMap hardwareMap) {
        horizontalSlidesMotor = hardwareMap.get(DcMotorEx.class,"horizontalSlidesMotor");
        horizontalSlidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalSlidesMotor.setDirection(DcMotor.Direction.REVERSE);
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

    public double getPosition() {
        return position;
    }
    public State getState() {
        return state;
    }

    public void update() {
        if (state == State.runToPosition) {
            horizontalSlidesMotor.setPower(pidController.update(horizontalSlidesMotor.getCurrentPosition() - position));
            // Brake if already in position

        }
        if(outputDebugInfo) {
            FtcDashboard dashboard = FtcDashboard.getInstance();
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("HorizontalSlides/Current", horizontalSlidesMotor.getCurrentPosition());
            packet.put("HorizontalSlides/Target", position);
            packet.put("HorizontalSlides/State", state);
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
