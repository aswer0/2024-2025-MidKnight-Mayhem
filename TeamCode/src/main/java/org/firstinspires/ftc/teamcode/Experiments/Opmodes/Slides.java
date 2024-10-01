package org.firstinspires.ftc.teamcode.Experiments.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class Slides extends OpMode {
    DcMotorEx FL;

    @Override
    public void init() {
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        FL.setPower(-gamepad1.left_stick_y);

        telemetry.addData("Encoder Ticks", FL.getCurrentPosition());
    }
}
