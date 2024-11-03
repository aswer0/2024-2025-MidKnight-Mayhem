package org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDController;

import java.util.concurrent.CompletableFuture;

public class HorizontalSlides {
    final static int MIN = 0;
    final static int MAX = 0;

    public DcMotorEx horizontalSlidesMotor;
    public PIDController pidController = new PIDController(0.001, 0, 0.001);
    private enum State {
        userControlled,
        runToPosition
    }
    private double position = 0;
    private State state = State.userControlled;

    public HorizontalSlides (HardwareMap hardwareMap) {
        horizontalSlidesMotor = hardwareMap.get(DcMotorEx.class,"horizontalSlidesMotor");
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
        horizontalSlidesMotor.setPower(power);
    }
    public void trySetPower(double power) {
        int slidePos = horizontalSlidesMotor.getCurrentPosition();
        if (slidePos<MAX && power>0) {
            setPower(power);
        } else if (slidePos>MIN && power<0) {
            setPower(power);
        }
    }
    public void update() {
        if (state == State.runToPosition) {
            horizontalSlidesMotor.setPower(pidController.calculate(horizontalSlidesMotor.getCurrentPosition(), position));
            // Brake if already in position

        }
    }

}
