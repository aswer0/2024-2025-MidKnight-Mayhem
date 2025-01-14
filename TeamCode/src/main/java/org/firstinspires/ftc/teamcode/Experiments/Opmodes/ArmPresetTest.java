package org.firstinspires.ftc.teamcode.Experiments.Opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Arm;

@TeleOp
@Config
public class ArmPresetTest extends OpMode {
    Arm arm;
    public static ArmPreset state = ArmPreset.intakeSample;
    @Override
    public void init() {
        arm = new Arm(hardwareMap);
    }

    @Override
    public void loop() {
        switch(state) {
            case intakeSample:
                arm.intakeSample();
                break;
            case intakeSpecimen:
                arm.intakeSpecimen();
                break;
            case outtakeSample:
                arm.outtakeSample();
                break;
            case outtakeSpecimen:
                arm.outtakeSpecimen();
                break;
        }
    }
    public enum ArmPreset {
        intakeSpecimen,
        intakeSample,
        outtakeSpecimen,
        outtakeSample
    }
}
