package org.firstinspires.ftc.teamcode.Experiments.Opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FinalPrograms.Subsystems.Outtake.Arm;
import org.firstinspires.ftc.teamcode.FinalPrograms.Subsystems.Outtake.Lift;

@TeleOp
@Config
public class ArmPresetTest extends OpMode {
    Arm arm;
    Lift outtakeSlides;
    public static ArmPreset state = ArmPreset.intakeSample;
    @Override
    public void init() {
        arm = new Arm(hardwareMap);
        outtakeSlides = new Lift(hardwareMap, true);
    }

    @Override
    public void loop() {
        outtakeSlides.update();
        if (gamepad1.right_bumper) {
            arm.openClaw();
        } else {
            arm.closeClaw();
        }

        if (gamepad1.dpad_down) {
            state = ArmPreset.intakeSpecimen;
        } else if (gamepad1.dpad_left) {
            state = ArmPreset.outtakeSpecimen1;
        } else if (gamepad1.dpad_right) {
            state = ArmPreset.outtakeSpecimen2;
        }

        switch(state) {
            case intakeSample:
                arm.intakeSample();
                break;
            case intakeSpecimen:
                arm.intakeSpecimen();
                break;
            case outtakeSample:

                arm.outtakeSample();
                outtakeSlides.toHighBasket();
                break;
            case outtakeSpecimen1:
                arm.outtakeSpecimen1();
                break;
            case outtakeSpecimen2:
                arm.outtakeSpecimen2();
                break;
            case toIdlePosition:
                arm.toIdlePosition();
                break;
            case toAutoStartPosition:
                arm.toAutoStartPosition();
                break;
            case backOuttakeSpecimen1:
                arm.backOuttakeSpecimen1();
                outtakeSlides.toBackChamber1Pos();
                break;
            case backOuttakeSpecimen2:
                arm.backOuttakeSpecimen2();
                outtakeSlides.toBackChamber2Pos();
        }
    }
    public enum ArmPreset {
        intakeSpecimen,
        intakeSample,
        outtakeSpecimen1,
        outtakeSpecimen2,
        outtakeSample,
        toIdlePosition,
        toAutoStartPosition,
        backOuttakeSpecimen1,
        backOuttakeSpecimen2
    }
}
