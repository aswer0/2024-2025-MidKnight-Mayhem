package org.firstinspires.ftc.teamcode.Experiments.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Lift;

@TeleOp
public class OuttakeTest extends OpMode {
    Lift lift;

    @Override
    public void init() {
        lift = new Lift(hardwareMap);
    }

    @Override
    public void loop() {
        lift.setPower(-gamepad1.left_stick_y);
    }
}
