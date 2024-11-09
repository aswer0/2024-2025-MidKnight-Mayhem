package org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Outtake {
    Lift lift;
    Manipulator manipulator;
    ElapsedTime outtakeTimer;

    public enum outtakeStates {
        ready,
        holdingSpecimen,
        highBasket,
        lowBasket,
        highRung,
        lowRung,
        lowered
    }

    public void doOuttake(outtakeStates outtakeState, Gamepad gamepad) {
        switch (outtakeState) {
            case ready:
                if (gamepad.left_trigger>0.5) {
                    //auto specimen claw (using color sensor)
                }

                break;
            case holdingSpecimen:
                break;
            case highBasket:
                break;
            case lowBasket:
                break;
            case highRung:
                break;
            case lowRung:
                break;
            case lowered:
                break;
        }
    }
    public static void lowerClaw(Lift lift, Manipulator manipulator) {
        lift.setPosition(100);
        manipulator.openClaw();
    }
}
