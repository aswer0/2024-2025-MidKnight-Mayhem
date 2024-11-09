package org.firstinspires.ftc.teamcode.Experiments.Opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class ServoTest extends OpMode {
    public static double test = 0.5;
    Servo servo;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "claw");
    }

    @Override
    public void init_loop() {
        loop();
    }

    @Override
    public void loop() {
        servo.setPosition(test);
        telemetry.addData("test", servo.getPortNumber());
    }
}
