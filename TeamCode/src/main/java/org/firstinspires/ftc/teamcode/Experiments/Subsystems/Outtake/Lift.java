package org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake.HorizontalSlides;
import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDFController;
import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDFControllerTest;


public class Lift {
    public DcMotorEx leftSlide;
    public DcMotorEx rightSlide;


    public static PIDFCoefficients coefficients = new PIDFCoefficients(-0.02,0,-0.001,0.1);

    public PIDFController leftMotorController = new PIDFController(coefficients);;

    private enum State {
        userControlled,
        runToPosition
    }
    private double position = 0;
    private State state = State.userControlled;

    public Lift (HardwareMap hardwareMap) {
        this.leftSlide = hardwareMap.get(DcMotorEx.class,"SlideRight"); //The one with the encoder
        this.leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        this.rightSlide = hardwareMap.get(DcMotorEx.class,"SlideLeft");
        this.rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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
    public void setPosition(double position, boolean changeState) {
        if(changeState) state = State.runToPosition;
        this.position = position;
    }
    public void setPosition(double position) {
        setPosition(position, true);
    }
    public void setPower(double power) {
        state = State.userControlled;
        leftSlide.setPower(power);
        rightSlide.setPower(-power);
    }
    public void update() {
        if (state == State.runToPosition) {
            double input = leftMotorController.update(leftSlide.getCurrentPosition() - position);
            leftSlide.setPower(leftMotorController.update(leftSlide.getCurrentPosition() - position));
            rightSlide.setPower(input);
            // Power should already be set for setPower
        }
    }

}
