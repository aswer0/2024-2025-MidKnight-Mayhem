package org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDController;

// todo PID
public class HorizontalSlides {
    public DcMotorEx horizontalSlidesMotor;
    public PIDController pidController = new PIDController(0.001, 0, 0.001);
    enum State {
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
    public void update() {
        if (state == State.runToPosition) {
            horizontalSlidesMotor.setPower(pidController.calculate(horizontalSlidesMotor.getCurrentPosition(), position));
            // Power should already be set for setPower
        }
    }

}
