package org.firstinspires.ftc.teamcode.Experiments.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ClawTest extends OpMode {
    Servo claw;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();


    @Override
    public void init() {
        claw = hardwareMap.get(Servo.class, "claw");
    }

    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) claw.setPosition(claw.getPosition()-0.05);
        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) claw.setPosition(claw.getPosition()+0.05);

        if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) claw.setPosition(0);
        if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) claw.setPosition(0.35);

        telemetry.addData("clawPos", claw.getPosition());
    }
}
