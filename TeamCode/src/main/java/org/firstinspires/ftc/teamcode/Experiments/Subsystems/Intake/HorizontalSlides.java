package org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDController;
import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDFController;

import java.util.concurrent.CompletableFuture;

@Config
public class HorizontalSlides {
    final static int MIN = 0;
    final static int MAX = 100;

    public DcMotorEx horizontalSlidesMotor;
    public static PIDFCoefficients coefficients = new PIDFCoefficients(-0.001,0,-0.001,0);
    public PIDFController pidController = new PIDFController(coefficients);
    private enum State {
        userControlled,
        runToPosition
    }
    private double position = 0;
    private State state = State.userControlled;

    public HorizontalSlides (HardwareMap hardwareMap) {
        horizontalSlidesMotor = hardwareMap.get(DcMotorEx.class,"horizontalSlidesMotor");
        horizontalSlidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

    //set power with limits
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
            horizontalSlidesMotor.setPower(pidController.update(horizontalSlidesMotor.getCurrentPosition() - position));
            // Brake if already in position

        }
    }
}
