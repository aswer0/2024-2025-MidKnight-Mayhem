package org.firstinspires.ftc.teamcode.Experiments.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Slides extends OpMode {
    DcMotorEx FL;

    Servo claw;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void init() {
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        claw = hardwareMap.get(Servo.class, "claw");
    }

    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        if (FL.getCurrentPosition()<=5) FL.setPower(0.1);
        if (FL.getCurrentPosition()>=465) FL.setPower(-0.1);
        if (FL.getCurrentPosition()>=0 && gamepad1.left_stick_y>0) {
            FL.setPower(-gamepad1.left_stick_y);
        } else if (FL.getCurrentPosition()<=460 && gamepad1.left_stick_y<0) {
            FL.setPower(-gamepad1.left_stick_y);
        } else {
            FL.setPower(0);
        }

        if (currentGamepad1.cross && !previousGamepad1.cross) claw.setPosition(0.35);
        if (currentGamepad1.circle && !previousGamepad1.circle) claw.setPosition(0.6);

        telemetry.addData("Encoder Ticks", FL.getCurrentPosition());
    }
}
